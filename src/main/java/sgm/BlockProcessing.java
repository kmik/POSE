package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.time.LocalDate;
import java.util.*;

/**
 * Opt-in block processing ({@code --block-processing on|auto}) for image sets that do not fit
 * this machine's memory as one block. Default off: nothing here runs unless requested.
 *
 * <ol>
 *   <li>Forecast: peak alignment and image-fusion memory as a function of the block size, from
 *       the image count, --features, --quality and the RAM this machine has free; the largest
 *       block that fits (with margin) is the block size. auto runs the ordinary single-block
 *       pipeline when everything fits.
 *   <li>Partition: the images' GPS positions are split recursively at the median (k-d) into
 *       cores of at most N images; each block also takes the images within --block-overlap image
 *       footprints of its core, so blocks share cameras and seams have full matching support.
 *   <li>Calibration once: the central block runs inputs..georeference; its final applied
 *       calibration (camera.txt) is given to every block as a fixed --calibration, so the blocks
 *       share one lens model. A user-supplied --calibration is used instead.
 *   <li>Blocks: each block is an ordinary POSE project under PROJECT/blocks, run as a child
 *       process with the user's arguments. Finished blocks are skipped when the command is
 *       repeated (resume).
 *   <li>Agreement: every camera shared by two blocks has two georeferenced positions. A small 3D
 *       similarity per block (translation, rotation, scale about the block centre) is solved by
 *       least squares over all shared cameras, with weak priors that keep each block at its GPS
 *       placement; seam residuals are reported before and after.
 *   <li>Merge: every point is kept only by the block whose own (core) camera is nearest in plan
 *       view, after the block's transform; merged dense.las, cameras.csv and a report are written
 *       to PROJECT/merged.
 * </ol>
 */
final class BlockProcessing {
  private BlockProcessing() {}

  private static final List<String> OWN = List.of("--block-processing", "--block-images", "--block-overlap");

  /** --block-processing value from raw arguments (off when absent). */
  static String mode(String[] args) {
    String mode = "off";
    for (int i = 0; i < args.length; i++) {
      if (args[i].startsWith("--block-processing=")) mode = args[i].substring(19);
      else if (args[i].equals("--block-processing") && i + 1 < args.length) mode = args[i + 1];
    }
    return mode.toLowerCase(Locale.ROOT);
  }

  static boolean isBlockProject(Path root) {
    return Files.isRegularFile(root.resolve("blocks").resolve("plan.json"));
  }

  record Image(Path path, String name, double x, double y, double footprint) {}

  record Block(int id, double[] core, List<Integer> images, List<Integer> home) {}

  /** blockImages: core images per block; maxImages: largest block (core + overlap) that fits. */
  record Forecast(int blockImages, int maxImages, long heapBytes, long alignment, long fusion, String reason) {}

  // ---------------------------------------------------------------------------------------------
  // Entry point

  /** Runs block processing; returns null when auto finds that one block fits (run as usual). */
  static Integer run(String[] args) throws IOException, InterruptedException {
    Options o = Options.parse(args);
    String mode = o.blockProcessing;
    List<Path> paths = Main.images(o);
    if (paths.size() < 2) throw new IllegalArgumentException("Block processing needs at least two images");
    Forecast forecast = forecast(o, paths);
    System.out.printf(
        Locale.ROOT,
        "Block processing forecast: %d images; about %.1f GB heap per process; one block of %d"
            + " images needs ~%.1f GB for alignment and ~%.1f GB for image fusion -> %s%n",
        paths.size(), forecast.heapBytes() / 1e9, paths.size(),
        estimateAlignment(o, paths.size()) / 1e9, estimateFusion(o, paths, paths.size()) / 1e9,
        forecast.reason());
    // Blocks bound the per-camera maps of all cameras, not one camera's own fusion working set
    // (its pairs' candidates and rasters). If even a two-image block does not fit, smaller blocks
    // cannot help: stop before any work instead of running blocks that will fail at fusion.
    long smallest = estimateFusion(o, paths, 2), allowance = forecast.heapBytes() * 7 / 10;
    if (smallest > allowance)
      throw new IllegalArgumentException(String.format(Locale.ROOT,
          "at --quality %s, fusing even a single camera needs about %.1f GB of heap (its stereo pairs'"
              + " working set), more than the ~%.1f GB a process can use here; smaller blocks cannot"
              + " help. Use a lower --quality (e.g. high), --fusion-mode reference, or free RAM.",
          o.quality, smallest / 1e9, allowance / 1e9));
    int size = o.blockImages > 0 ? o.blockImages : forecast.blockImages();
    if (mode.equals("auto") && o.blockImages == 0 && size >= paths.size()) {
      Path existing = Main.projectRoot(args);
      if (isBlockProject(existing) && !Files.isRegularFile(existing.resolve("project.properties")))
        throw new IllegalArgumentException("the images fit one block, so this would be an ordinary run, but "
            + existing + " holds an earlier block-processing run (blocks/plan.json). Use a new output"
            + " folder, or delete that one.");
      System.out.println("Block processing (auto): the whole image set fits one block; running as usual.");
      return null;
    }
    Path root = Main.projectRoot(args);
    Files.createDirectories(root);
    List<Image> images = inventory(paths);
    // A block (core + overlap) never exceeds what fits in memory, and at most 2x its core.
    int maxImages = Math.min(forecast.maxImages(), Math.max(2, 2 * size));
    if (o.blockImages > 0 && o.blockImages > forecast.maxImages())
      System.out.println("Warning: --block-images " + o.blockImages + " exceeds the " + forecast.maxImages()
          + " images the memory forecast allows per block");
    List<Block> blocks = partition(images, Math.max(2, size), o.blockOverlap, Math.max(maxImages, size));
    Path blocksDir = root.resolve("blocks");
    Files.createDirectories(blocksDir);
    writePlan(blocksDir.resolve("plan.json"), o, images, blocks, size, forecast);
    // Blocks are aligned to each other through shared cameras; without them a block keeps only its
    // own GPS placement and its seams can be off by about the GPS accuracy.
    for (Block b : blocks) {
      int shared = 0;
      for (Block other : blocks)
        if (other != b) for (int i : b.images()) if (other.images().contains(i)) shared++;
      if (blocks.size() > 1 && shared < 3)
        System.out.println("Warning: block " + b.id() + " shares " + shared + " cameras with its neighbours"
            + " (memory limited its overlap); its seams rely on GPS alone. Larger blocks (more free RAM,"
            + " lower --quality) give more shared cameras.");
    }
    System.out.printf(
        Locale.ROOT, "Block processing: %d images in %d blocks (at most %d core images each, overlap %.1f footprints)%n",
        images.size(), blocks.size(), size, o.blockOverlap);
    List<String> base = baseArguments(args);
    Path calibration = o.calibration;
    if (calibration == null) calibration = calibrate(blocksDir, images, blocks, base);
    for (Block b : blocks) runBlock(blocksDir, images, b, blocks.size(), base, calibration);
    merge(root, images, blocks);
    return 0;
  }

  // ---------------------------------------------------------------------------------------------
  // Forecast

  /** Bytes of the image-fusion heap for a block of n images (ImageFusion's own admission rule). */
  static long estimateFusion(Options o, List<Path> paths, int n) throws IOException {
    if (!o.fusionMode.equals("image")) return 0;
    int[] size = o.processedSize(ResolutionPlan.originalSize(paths.get(0))[0], ResolutionPlan.originalSize(paths.get(0))[1]);
    long pixels = (long) size[0] * size[1];
    boolean filter = DepthOutlierFilter.enabled(o);
    long perCamera = (filter ? 8 : 4) * pixels + (o.fusionLocalSigma > 0 ? 4 * pixels : 0);
    long cache = filter ? 4096L * SurfacePatchGrid.cacheCells(o, n) : 0;
    int words = (n + 63) / 64, pairsPerCamera = Math.max(1, 2 * o.neighbors);
    // Candidates per pair: about a third of the pixels survive stereo; 64 bytes per candidate.
    long worst = pairsPerCamera * (pixels / 3 * 64 + 20 * pixels) + pixels * (34L + 8L * words);
    return n * (perCamera + cache) + worst;
  }

  /** Bytes of the sparse alignment graph and bundle workspace for a block of n images. */
  static long estimateAlignment(Options o, int n) {
    long keypoints = (long) n * o.features, observations = keypoints / 2, tracks = observations / 3;
    long pairs = (long) n * Math.max(1, o.neighbors), matches = pairs * o.features / 4;
    return AlignmentMemory.graphBytes(keypoints, tracks, observations, pairs, matches)
        + AlignmentMemory.bundleBytes(tracks, observations, n, true);
  }

  /** Largest block (images) whose alignment and fusion fit 70% of the heap a process gets here. */
  static Forecast forecast(Options o, List<Path> paths) throws IOException {
    MemorySafety.Limits limits = MemorySafety.limits(o.ramLimitMb);
    long available = limits.available() > 0 ? limits.available() : Runtime.getRuntime().maxMemory();
    // The launcher gives a process at most 65% of the available RAM as heap.
    long heap = Math.max(256L << 20, available / 100 * 65);
    long allowance = heap * 7 / 10;
    int n = paths.size();
    if (estimateAlignment(o, n) <= allowance && estimateFusion(o, paths, n) <= allowance)
      return new Forecast(n, n, heap, estimateAlignment(o, n), estimateFusion(o, paths, n), "fits one block");
    int lo = 2, hi = n;
    while (lo < hi) {
      int mid = (lo + hi + 1) / 2;
      if (estimateAlignment(o, mid) <= allowance && estimateFusion(o, paths, mid) <= allowance) lo = mid;
      else hi = mid - 1;
    }
    // Overlap images also belong to a block: leave room for about 50% more images than its core.
    int core = Math.max(8, lo * 2 / 3);
    return new Forecast(core, lo, heap, estimateAlignment(o, lo), estimateFusion(o, paths, lo),
        "blocks of at most " + core + " core images");
  }

  // ---------------------------------------------------------------------------------------------
  // Inventory and partition

  static List<Image> inventory(List<Path> paths) throws IOException {
    List<Exif.Metadata> meta = new ArrayList<>();
    double lat0 = 0, lon0 = 0;
    for (Path p : paths) {
      Exif.Metadata m = Exif.read(p);
      if (m == null || m.gps == null || !m.gps.usable())
        throw new IllegalArgumentException("Block processing needs GPS positions in every image; missing in " + p.getFileName());
      meta.add(m);
      lat0 += m.gps.latitude / paths.size();
      lon0 += m.gps.longitude / paths.size();
    }
    double kx = 111320 * Math.cos(Math.toRadians(lat0)), ky = 110540;
    List<double[]> xy = new ArrayList<>();
    for (Exif.Metadata m : meta) xy.add(new double[] {(m.gps.longitude - lon0) * kx, (m.gps.latitude - lat0) * ky});
    // Ground footprint (longer image side): relative altitude x 36 mm / 35 mm-equivalent focal.
    List<Double> footprints = new ArrayList<>();
    for (Exif.Metadata m : meta)
      if (Double.isFinite(m.relativeAltitude) && m.relativeAltitude > 1 && m.focal35Mm > 0)
        footprints.add(m.relativeAltitude * 36 / m.focal35Mm);
    double footprint;
    if (!footprints.isEmpty()) {
      Collections.sort(footprints);
      footprint = footprints.get(footprints.size() / 2);
    } else footprint = 4 * medianSpacing(xy);
    List<Image> out = new ArrayList<>();
    for (int i = 0; i < paths.size(); i++)
      out.add(new Image(paths.get(i), paths.get(i).getFileName().toString(), xy.get(i)[0], xy.get(i)[1], footprint));
    return out;
  }

  static double medianSpacing(List<double[]> xy) {
    List<Double> nearest = new ArrayList<>();
    for (int i = 0; i < xy.size(); i++) {
      double best = Double.POSITIVE_INFINITY;
      for (int j = 0; j < xy.size(); j++)
        if (i != j) best = Math.min(best, Math.hypot(xy.get(i)[0] - xy.get(j)[0], xy.get(i)[1] - xy.get(j)[1]));
      if (Double.isFinite(best)) nearest.add(best);
    }
    Collections.sort(nearest);
    return nearest.isEmpty() ? 10 : Math.max(1, nearest.get(nearest.size() / 2));
  }

  /** k-d split of the positions into cores of at most size images; blocks add overlap images. */
  static List<Block> partition(List<Image> images, int size, double overlap) {
    return partition(images, size, overlap, Integer.MAX_VALUE);
  }

  /**
   * As above; a block's overlap margin is halved (down to the core alone) until the block has at
   * most maxImages images, so overlap never defeats the memory limit the blocks exist for.
   */
  static List<Block> partition(List<Image> images, int size, double overlap, int maxImages) {
    List<double[]> cores = new ArrayList<>();
    List<List<Integer>> members = new ArrayList<>();
    List<Integer> all = new ArrayList<>();
    for (int i = 0; i < images.size(); i++) all.add(i);
    split(images, all, new double[] {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY}, size, cores, members);
    List<Block> blocks = new ArrayList<>();
    for (int b = 0; b < cores.size(); b++) {
      double[] c = cores.get(b);
      double margin = overlap * images.get(0).footprint();
      List<Integer> inBlock;
      while (true) {
        inBlock = new ArrayList<>();
        for (int i = 0; i < images.size(); i++) {
          Image im = images.get(i);
          if (im.x() >= c[0] - margin && im.x() <= c[2] + margin && im.y() >= c[1] - margin && im.y() <= c[3] + margin)
            inBlock.add(i);
        }
        if (inBlock.size() <= maxImages || margin == 0) break;
        margin = margin < 1 ? 0 : margin / 2;
      }
      blocks.add(new Block(b, c, inBlock, members.get(b)));
    }
    return blocks;
  }

  private static void split(List<Image> images, List<Integer> ids, double[] box, int size,
      List<double[]> cores, List<List<Integer>> members) {
    if (ids.size() <= size) {
      cores.add(box);
      members.add(ids);
      return;
    }
    double x0 = Double.MAX_VALUE, x1 = -Double.MAX_VALUE, y0 = Double.MAX_VALUE, y1 = -Double.MAX_VALUE;
    for (int i : ids) {
      x0 = Math.min(x0, images.get(i).x()); x1 = Math.max(x1, images.get(i).x());
      y0 = Math.min(y0, images.get(i).y()); y1 = Math.max(y1, images.get(i).y());
    }
    boolean alongX = x1 - x0 >= y1 - y0;
    List<Integer> sorted = new ArrayList<>(ids);
    sorted.sort(Comparator.comparingDouble((Integer i) -> alongX ? images.get(i).x() : images.get(i).y())
        .thenComparing(i -> images.get(i).name()));
    int half = sorted.size() / 2;
    double cut = alongX
        ? (images.get(sorted.get(half - 1)).x() + images.get(sorted.get(half)).x()) / 2
        : (images.get(sorted.get(half - 1)).y() + images.get(sorted.get(half)).y()) / 2;
    double[] first = box.clone(), second = box.clone();
    if (alongX) { first[2] = cut; second[0] = cut; } else { first[3] = cut; second[1] = cut; }
    split(images, new ArrayList<>(sorted.subList(0, half)), first, size, cores, members);
    split(images, new ArrayList<>(sorted.subList(half, sorted.size())), second, size, cores, members);
  }

  private static void writePlan(Path file, Options o, List<Image> images, List<Block> blocks, int size,
      Forecast forecast) throws IOException {
    List<Object> list = new ArrayList<>();
    for (Block b : blocks) {
      List<String> names = new ArrayList<>(), home = new ArrayList<>();
      for (int i : b.images()) names.add(images.get(i).name());
      for (int i : b.home()) home.add(images.get(i).name());
      list.add(Json.object("block", b.id(), "core_local_metres", finite(b.core()), "images", names, "core_images", home));
    }
    Json.write(file, Json.object(
        "schema", "pose-block-plan-1",
        "mode", o.blockProcessing,
        "images", images.size(),
        "core_images_per_block", size,
        "overlap_footprints", o.blockOverlap,
        "footprint_metres", images.get(0).footprint(),
        "forecast", Json.object("heap_bytes_per_process", forecast.heapBytes(),
            "alignment_bytes_one_block", forecast.alignment(), "fusion_bytes_one_block", forecast.fusion(),
            "decision", forecast.reason()),
        "blocks", list));
  }

  private static List<Double> finite(double[] box) {
    List<Double> out = new ArrayList<>();
    for (double v : box) out.add(Double.isFinite(v) ? v : null);
    return out;
  }

  // ---------------------------------------------------------------------------------------------
  // Child runs

  /** User arguments without -i/-o and the block options. */
  static List<String> baseArguments(String[] args) {
    List<String> out = new ArrayList<>();
    for (int i = 0; i < args.length; i++) {
      String a = args[i], key = a.contains("=") ? a.substring(0, a.indexOf('=')) : a;
      boolean ownWithValue = OWN.contains(key) || key.equals("-i") || key.equals("--images")
          || key.equals("-o") || key.equals("--output");
      if (ownWithValue) {
        if (!a.contains("=")) i++;
        continue;
      }
      out.add(a);
    }
    return out;
  }

  private static Path linkImages(Path dir, List<Image> images, List<Integer> ids) throws IOException {
    Files.createDirectories(dir);
    // The folder must hold exactly this block's images (a re-plan may have changed the set).
    Set<String> wanted = new HashSet<>();
    for (int i : ids) wanted.add(images.get(i).name());
    try (var existing = Files.list(dir)) {
      for (Path p : existing.toList()) if (!wanted.contains(p.getFileName().toString())) Files.delete(p);
    }
    for (int i : ids) {
      Path target = dir.resolve(images.get(i).name()), source = images.get(i).path().toAbsolutePath();
      if (Files.exists(target)) continue;
      try {
        Files.createLink(target, source);
      } catch (IOException | UnsupportedOperationException e) {
        try {
          Files.createSymbolicLink(target, source);
        } catch (IOException | UnsupportedOperationException e2) {
          Files.copy(source, target);
        }
      }
    }
    return dir;
  }

  private static int child(List<String> arguments, Path log) throws IOException, InterruptedException {
    List<String> command = new ArrayList<>();
    command.add(Path.of(System.getProperty("java.home"), "bin", "java").toString());
    command.add("--enable-native-access=ALL-UNNAMED");
    command.add("-cp");
    command.add(System.getProperty("java.class.path"));
    command.add("photogrammetry");
    command.addAll(arguments);
    System.out.println("Block processing: photogrammetry " + String.join(" ", arguments));
    Process p = new ProcessBuilder(command).inheritIO().start();
    int code = p.waitFor();
    Files.writeString(log, Json.object("exit_code", code, "arguments", arguments).toString(), StandardCharsets.UTF_8);
    return code;
  }

  private static Path latestRun(Path project) throws IOException {
    return project.resolve("runs").resolve(Main.latestRun(project));
  }

  /** Calibration once: the central block runs inputs..georeference; its final camera.txt. */
  private static Path calibrate(Path blocksDir, List<Image> images, List<Block> blocks, List<String> base)
      throws IOException, InterruptedException {
    Path dir = blocksDir.resolve("calibration"), done = dir.resolve("camera.txt");
    if (Files.isRegularFile(done)) {
      System.out.println("Block processing: reusing the shared calibration " + done);
      return done;
    }
    double cx = 0, cy = 0;
    for (Image im : images) { cx += im.x() / images.size(); cy += im.y() / images.size(); }
    Block central = blocks.get(0);
    double best = Double.MAX_VALUE;
    for (Block b : blocks) {
      double mx = 0, my = 0;
      for (int i : b.home()) { mx += images.get(i).x() / b.home().size(); my += images.get(i).y() / b.home().size(); }
      double d = Math.hypot(mx - cx, my - cy);
      if (d < best) { best = d; central = b; }
    }
    Path input = linkImages(dir.resolve("images"), images, central.images()), project = dir.resolve("project");
    List<String> first = new ArrayList<>(List.of("inputs"));
    first.addAll(base);
    first.addAll(List.of("-i", input.toString(), "-o", project.toString()));
    if (child(first, dir.resolve("inputs.json")) != 0) throw new IOException("Calibration block: inputs step failed");
    List<String> rest = new ArrayList<>(List.of("calibrate", "match", "align", "georeference", "-o", project.toString()));
    if (child(rest, dir.resolve("steps.json")) != 0) throw new IOException("Calibration block: alignment failed");
    Path camera = latestRun(project).resolve("camera.txt");
    if (!Files.isRegularFile(camera)) throw new IOException("Calibration block wrote no camera.txt: " + camera);
    Files.copy(camera, done, StandardCopyOption.REPLACE_EXISTING);
    System.out.println("Block processing: shared calibration from block " + central.id() + " -> " + done);
    return done;
  }

  private static Path blockDir(Path blocksDir, int id) {
    return blocksDir.resolve(String.format(Locale.ROOT, "block_%03d", id));
  }

  private static void runBlock(Path blocksDir, List<Image> images, Block b, int total, List<String> base,
      Path calibration) throws IOException, InterruptedException {
    Path dir = blockDir(blocksDir, b.id()), done = dir.resolve("done.json");
    List<String> names = new ArrayList<>();
    for (int i : b.images()) names.add(images.get(i).name());
    String signature = String.join("|", names);
    Path images_ = dir.resolve("done-images.txt");
    if (Files.isRegularFile(done) && Files.isRegularFile(images_) && Files.readString(images_).equals(signature)) {
      System.out.println("Block processing: block " + b.id() + " already complete; skipped");
      return;
    }
    Path input = linkImages(dir.resolve("images"), images, b.images()), project = dir.resolve("project");
    List<String> args = new ArrayList<>(base);
    args.addAll(List.of("-i", input.toString(), "-o", project.toString(), "--calibration", calibration.toString()));
    System.out.printf(Locale.ROOT, "Block processing: block %d of %d (%d images, %d core)%n",
        b.id() + 1, total, b.images().size(), b.home().size());
    if (child(args, dir.resolve("last-attempt.json")) != 0)
      throw new IOException("Block " + b.id() + " failed; rerun the same command to resume from it");
    Path run = latestRun(project);
    if (!Files.isRegularFile(run.resolve("07_results/dense.las")))
      throw new IOException("Block " + b.id() + " finished without 07_results/dense.las");
    Files.writeString(dir.resolve("done-images.txt"), signature);
    Json.write(done, Json.object("block", b.id(), "run", run.toString(), "images", names.size()));
  }

  // ---------------------------------------------------------------------------------------------
  // Agreement between blocks

  record Pose(double[] center, double[] rotation) {}

  static Map<String, Pose> exterior(Path csv) throws IOException {
    Map<String, Pose> out = new LinkedHashMap<>();
    List<String> lines = Files.readAllLines(csv);
    for (String line : lines.subList(1, lines.size())) {
      List<String> f = PoseIO.csv(line);
      if (!f.get(2).equals("true")) continue;
      double[] r = new double[9];
      for (int k = 0; k < 9; k++) r[k] = Double.parseDouble(f.get(4 + k));
      out.put(f.get(1), new Pose(new double[] {Double.parseDouble(f.get(16)), Double.parseDouble(f.get(17)),
          Double.parseDouble(f.get(18))}, r));
    }
    return out;
  }

  /** Similarity (about centre c): x' = c + (1 + s) R(w) (x - c) + t. */
  static final class Similarity {
    private final double[] c, t, w, r;
    private final double s;

    Similarity(double[] c, double[] t, double[] w, double s) {
      this.c = c; this.t = t; this.w = w; this.s = s;
      r = rodrigues(w);
    }

    /** Rotation matrix of a rotation vector (pure Java: the driver process loads no OpenCV). */
    static double[] rodrigues(double[] w) {
      double angle = Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
      if (angle < 1e-15) return new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1};
      double x = w[0] / angle, y = w[1] / angle, z = w[2] / angle, c = Math.cos(angle), s = Math.sin(angle), v = 1 - c;
      return new double[] {
        c + x * x * v, x * y * v - z * s, x * z * v + y * s,
        y * x * v + z * s, c + y * y * v, y * z * v - x * s,
        z * x * v - y * s, z * y * v + x * s, c + z * z * v};
    }

    double[] t() { return t; }
    double[] w() { return w; }
    double s() { return s; }

    double[] apply(double[] x) {
      double dx = x[0] - c[0], dy = x[1] - c[1], dz = x[2] - c[2], f = 1 + s;
      return new double[] {
        c[0] + f * (r[0] * dx + r[1] * dy + r[2] * dz) + t[0],
        c[1] + f * (r[3] * dx + r[4] * dy + r[5] * dz) + t[1],
        c[2] + f * (r[6] * dx + r[7] * dy + r[8] * dz) + t[2]};
    }
  }

  /**
   * Linearized least squares over all cameras shared by two blocks: positions (5 cm per
   * coordinate) and orientations (0.02 deg), with priors keeping each block near its own GPS
   * placement (2 m, 0.5 deg, 0.5% scale).
   */
  static Similarity[] agree(List<Map<String, Pose>> poses) {
    int B = poses.size(), n = 7 * B;
    double[][] centres = centres(poses);
    double[] a = new double[n * n], rhs = new double[n];
    double wObs = 1 / (0.05 * 0.05), wRot = 1 / Math.pow(Math.toRadians(.02), 2);
    double[] prior = {1 / 4., 1 / 4., 1 / 4., 1 / Math.pow(Math.toRadians(.5), 2), 1 / Math.pow(Math.toRadians(.5), 2),
        1 / Math.pow(Math.toRadians(.5), 2), 1 / (0.005 * 0.005)};
    for (int b = 0; b < B; b++) for (int k = 0; k < 7; k++) a[(7 * b + k) * n + 7 * b + k] += prior[k];
    for (int i = 0; i < B; i++)
      for (int j = i + 1; j < B; j++)
        for (var e : poses.get(i).entrySet()) {
          Pose other = poses.get(j).get(e.getKey());
          if (other == null) continue;
          position(a, rhs, n, i, j, e.getValue().center(), other.center(), centres, wObs);
          // Orientation: the camera's world-to-camera rotation becomes R R(w)^T, so agreement
          // needs w_i - w_j = delta with [delta]x ~ R_j^T R_i - I. Camera centres of a nadir block
          // lie nearly in a plane and hardly constrain tilt; orientations do.
          double[] ri = e.getValue().rotation(), rj = other.rotation(), d = new double[9];
          for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
              for (int k = 0; k < 3; k++) d[3 * r + c] += rj[3 * k + r] * ri[3 * k + c];
          double[] delta = {(d[7] - d[5]) / 2, (d[2] - d[6]) / 2, (d[3] - d[1]) / 2};
          for (int k = 0; k < 3; k++) {
            int pi = 7 * i + 3 + k, pj = 7 * j + 3 + k;
            a[pi * n + pi] += wRot;
            a[pj * n + pj] += wRot;
            a[pi * n + pj] -= wRot;
            a[pj * n + pi] -= wRot;
            rhs[pi] += wRot * delta[k];
            rhs[pj] -= wRot * delta[k];
          }
        }
    double[] x = solve(a, rhs, n);
    Similarity[] out = new Similarity[B];
    for (int b = 0; b < B; b++)
      out[b] = new Similarity(centres[b], new double[] {x[7 * b], x[7 * b + 1], x[7 * b + 2]},
          new double[] {x[7 * b + 3], x[7 * b + 4], x[7 * b + 5]}, x[7 * b + 6]);
    return out;
  }

  static double[][] centres(List<Map<String, Pose>> poses) {
    double[][] centres = new double[poses.size()][3];
    for (int b = 0; b < poses.size(); b++)
      for (Pose p : poses.get(b).values())
        for (int k = 0; k < 3; k++) centres[b][k] += p.center()[k] / poses.get(b).size();
    return centres;
  }

  /** One point seen by blocks i (at xi) and j (at xj): x'_i - x'_j = 0, weight w per coordinate. */
  private static void position(double[] a, double[] rhs, int n, int i, int j, double[] xi, double[] xj,
      double[][] centres, double w) {
    double[] di = Math3.sub(xi, centres[i]), dj = Math3.sub(xj, centres[j]);
    for (int row = 0; row < 3; row++) {
      double[] ji = jacobianRow(di, row), jj = jacobianRow(dj, row);
      double r0 = xi[row] - xj[row];
      for (int p = 0; p < 7; p++) {
        rhs[7 * i + p] -= w * ji[p] * r0;
        rhs[7 * j + p] += w * jj[p] * r0;
        for (int q = 0; q < 7; q++) {
          a[(7 * i + p) * n + 7 * i + q] += w * ji[p] * ji[q];
          a[(7 * j + p) * n + 7 * j + q] += w * jj[p] * jj[q];
          a[(7 * i + p) * n + 7 * j + q] -= w * ji[p] * jj[q];
          a[(7 * j + p) * n + 7 * i + q] -= w * jj[p] * ji[q];
        }
      }
    }
  }

  /** d(x'_row)/d(t, w, s) for x' = x + t + w x d + s d. */
  private static double[] jacobianRow(double[] d, int row) {
    double[] j = new double[7];
    j[row] = 1;
    // (w x d)_row: row 0: w1 d2 - w2 d1; row 1: w2 d0 - w0 d2; row 2: w0 d1 - w1 d0.
    if (row == 0) { j[4] = d[2]; j[5] = -d[1]; }
    else if (row == 1) { j[3] = -d[2]; j[5] = d[0]; }
    else { j[3] = d[1]; j[4] = -d[0]; }
    j[6] = d[row];
    return j;
  }

  /** Symmetric positive definite solve (Cholesky). */
  static double[] solve(double[] a, double[] b, int n) {
    double[] l = new double[n * n];
    for (int i = 0; i < n; i++)
      for (int j = 0; j <= i; j++) {
        double sum = a[i * n + j];
        for (int k = 0; k < j; k++) sum -= l[i * n + k] * l[j * n + k];
        if (i == j) {
          if (!(sum > 0)) throw new IllegalStateException("Block agreement system is not positive definite");
          l[i * n + i] = Math.sqrt(sum);
        } else l[i * n + j] = sum / l[j * n + j];
      }
    double[] y = new double[n], x = new double[n];
    for (int i = 0; i < n; i++) {
      double sum = b[i];
      for (int k = 0; k < i; k++) sum -= l[i * n + k] * y[k];
      y[i] = sum / l[i * n + i];
    }
    for (int i = n - 1; i >= 0; i--) {
      double sum = y[i];
      for (int k = i + 1; k < n; k++) sum -= l[k * n + i] * x[k];
      x[i] = sum / l[i * n + i];
    }
    return x;
  }

  /** Shared cameras: count, RMS centre disagreement (m), RMS orientation disagreement (deg). */
  static double[] seam(Map<String, Pose> a, Map<String, Pose> b, Similarity ta, Similarity tb) {
    double sum = 0, angles = 0;
    int count = 0;
    double[] wa = ta == null ? new double[9] : transposed(Similarity.rodrigues(ta.w()));
    double[] wb = tb == null ? new double[9] : transposed(Similarity.rodrigues(tb.w()));
    for (var e : a.entrySet()) {
      Pose other = b.get(e.getKey());
      if (other == null) continue;
      double[] x = ta == null ? e.getValue().center() : ta.apply(e.getValue().center());
      double[] y = tb == null ? other.center() : tb.apply(other.center());
      sum += Math.pow(Math3.norm(Math3.sub(x, y)), 2);
      double[] ra = ta == null ? e.getValue().rotation() : multiply(e.getValue().rotation(), wa);
      double[] rb = tb == null ? other.rotation() : multiply(other.rotation(), wb);
      double trace = 0;
      for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) trace += rb[3 * k + r] * ra[3 * k + r];
      double angle = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2))));
      angles += angle * angle;
      count++;
    }
    return new double[] {count, count == 0 ? Double.NaN : Math.sqrt(sum / count), count == 0 ? Double.NaN : Math.sqrt(angles / count)};
  }

  private static double[] transposed(double[] m) {
    return new double[] {m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
  }

  private static double[] multiply(double[] p, double[] q) {
    double[] out = new double[9];
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        for (int k = 0; k < 3; k++) out[3 * r + c] += p[3 * r + k] * q[3 * k + c];
    return out;
  }

  // ---------------------------------------------------------------------------------------------
  // Merge

  /** Plan-view nearest-camera lookup over the core cameras of all blocks. */
  static final class Owners {
    final double[] x, y;
    final int[] block;
    final double cell, x0, y0;
    final Map<Long, List<Integer>> grid = new HashMap<>();

    Owners(List<double[]> centres, List<Integer> blocks) {
      int n = centres.size();
      x = new double[n]; y = new double[n]; block = new int[n];
      List<double[]> xy = new ArrayList<>();
      for (int i = 0; i < n; i++) {
        x[i] = centres.get(i)[0]; y[i] = centres.get(i)[1]; block[i] = blocks.get(i);
        xy.add(new double[] {x[i], y[i]});
      }
      cell = 2 * medianSpacing(xy);
      double mx = Double.MAX_VALUE, my = Double.MAX_VALUE;
      for (int i = 0; i < n; i++) { mx = Math.min(mx, x[i]); my = Math.min(my, y[i]); }
      x0 = mx; y0 = my;
      for (int i = 0; i < n; i++) grid.computeIfAbsent(key((int) Math.floor((x[i] - x0) / cell), (int) Math.floor((y[i] - y0) / cell)), k -> new ArrayList<>()).add(i);
    }

    private static long key(int gx, int gy) {
      return ((long) gx << 32) ^ (gy & 0xffffffffL);
    }

    int owner(double px, double py) {
      int gx = (int) Math.floor((px - x0) / cell), gy = (int) Math.floor((py - y0) / cell);
      double best = Double.POSITIVE_INFINITY;
      int found = -1;
      for (int ring = 0; ring < 100000; ring++) {
        for (int dy = -ring; dy <= ring; dy++)
          for (int dx = -ring; dx <= ring; dx++) {
            if (Math.max(Math.abs(dx), Math.abs(dy)) != ring) continue;
            List<Integer> ids = grid.get(key(gx + dx, gy + dy));
            if (ids == null) continue;
            for (int i : ids) {
              double d = (x[i] - px) * (x[i] - px) + (y[i] - py) * (y[i] - py);
              if (d < best) { best = d; found = i; }
            }
          }
        // Every cell outside this ring is at least ring * cell away.
        if (found >= 0 && Math.sqrt(best) <= ring * cell) break;
      }
      return found < 0 ? -1 : block[found];
    }
  }

  static void merge(Path root, List<Image> images, List<Block> blocks) throws IOException {
    Path blocksDir = root.resolve("blocks"), out = root.resolve("merged");
    Files.createDirectories(out);
    List<Path> runs = new ArrayList<>();
    List<Map<String, Pose>> poses = new ArrayList<>();
    for (Block b : blocks) {
      Path run = latestRun(blockDir(blocksDir, b.id()).resolve("project"));
      runs.add(run);
      poses.add(exterior(run.resolve("04_geometry/final/exterior.csv")));
    }
    // Cameras shared by blocks: positions and orientations (see agree).
    Similarity[] transforms = agree(poses);
    List<Object> seams = new ArrayList<>();
    for (int i = 0; i < blocks.size(); i++)
      for (int j = i + 1; j < blocks.size(); j++) {
        double[] before = seam(poses.get(i), poses.get(j), null, null), after = seam(poses.get(i), poses.get(j), transforms[i], transforms[j]);
        if (before[0] > 0)
          seams.add(Json.object("blocks", List.of(i, j), "shared_cameras", (int) before[0],
              "rms_before_metres", before[1], "rms_after_metres", after[1],
              "orientation_rms_before_degrees", before[2], "orientation_rms_after_degrees", after[2]));
      }
    // Core cameras (home block), transformed; cameras.csv lists every image from its home block.
    List<double[]> centres = new ArrayList<>();
    List<Integer> owners = new ArrayList<>();
    StringBuilder cameras = new StringBuilder("filename,block,registered,center_x,center_y,center_z\n");
    for (Block b : blocks)
      for (int i : b.home()) {
        Pose p = poses.get(b.id()).get(images.get(i).name());
        if (p == null) {
          cameras.append(PoseIO.quote(images.get(i).name())).append(',').append(b.id()).append(",false,,,\n");
          continue;
        }
        double[] c = transforms[b.id()].apply(p.center());
        centres.add(c);
        owners.add(b.id());
        cameras.append(PoseIO.quote(images.get(i).name())).append(',').append(b.id()).append(",true,")
            .append(c[0]).append(',').append(c[1]).append(',').append(c[2]).append('\n');
      }
    Files.writeString(out.resolve("cameras.csv"), cameras);
    if (centres.isEmpty()) throw new IOException("No block registered any of its core cameras");
    Owners lookup = new Owners(centres, owners);
    long[] kept = mergeLas(runs, transforms, lookup, out.resolve("dense.las"));
    List<Object> list = new ArrayList<>();
    for (Block b : blocks) {
      Similarity t = transforms[b.id()];
      list.add(Json.object("block", b.id(), "run", runs.get(b.id()).toString(), "images", b.images().size(),
          "core_images", b.home().size(), "registered", poses.get(b.id()).size(), "points_kept", kept[b.id()],
          "translation_metres", List.of(t.t()[0], t.t()[1], t.t()[2]),
          "rotation_degrees", Math.toDegrees(Math3.norm(t.w())), "scale", 1 + t.s()));
    }
    long total = 0;
    for (long k : kept) total += k;
    Json.write(out.resolve("block_report.json"), Json.object(
        "schema", "pose-block-merge-1", "blocks", list, "seams", seams, "merged_points", total,
        "ownership", "each point is kept by the block whose own (core) camera is nearest in plan view after transforms",
        "agreement", "one 3D similarity per block from cameras shared between blocks (5 cm per coordinate), with priors 2 m / 0.5 deg / 0.5% toward each block's own GPS placement"));
    StringBuilder md = new StringBuilder("# Block processing result\n\n");
    md.append(String.format(Locale.ROOT, "%d blocks, %d merged points (merged/dense.las).%n%n", blocks.size(), total));
    md.append("| block | images | core | registered | points kept | shift (m) | rotation (deg) | scale |\n|---:|---:|---:|---:|---:|---:|---:|---:|\n");
    for (Block b : blocks) {
      Similarity t = transforms[b.id()];
      md.append(String.format(Locale.ROOT, "| %d | %d | %d | %d | %d | %.3f | %.4f | %.5f |%n", b.id(), b.images().size(),
          b.home().size(), poses.get(b.id()).size(), kept[b.id()], Math3.norm(t.t()), Math.toDegrees(Math3.norm(t.w())), 1 + t.s()));
    }
    md.append("\n| seam | shared cameras | RMS before (m) | RMS after (m) | orientation before (deg) | after (deg) |\n|---|---:|---:|---:|---:|---:|\n");
    for (int i = 0; i < blocks.size(); i++)
      for (int j = i + 1; j < blocks.size(); j++) {
        double[] before = seam(poses.get(i), poses.get(j), null, null), after = seam(poses.get(i), poses.get(j), transforms[i], transforms[j]);
        if (before[0] > 0)
          md.append(String.format(Locale.ROOT, "| %d-%d | %d | %.3f | %.3f | %.4f | %.4f |%n", i, j, (int) before[0], before[1], after[1], before[2], after[2]));
      }
    Files.writeString(out.resolve("block_report.md"), md);
    System.out.println("Block processing: merged " + total + " points from " + blocks.size() + " blocks -> " + out.resolve("dense.las"));
    System.out.print(md);
  }

  private record Las(long offset, int recordLength, long count, double[] scale, double[] origin, byte[] vlrs, int vlrCount) {}

  private static Las header(Path file) throws IOException {
    try (FileChannel ch = FileChannel.open(file)) {
      ByteBuffer h = ByteBuffer.allocate(227).order(ByteOrder.LITTLE_ENDIAN);
      ch.read(h, 0);
      if (!new String(h.array(), 0, 4, StandardCharsets.US_ASCII).equals("LASF")) throw new IOException("Not a LAS file: " + file);
      int headerSize = h.getShort(94) & 0xffff;
      long offset = h.getInt(96) & 0xffffffffL;
      int vlrCount = h.getInt(100);
      int recordLength = h.getShort(105) & 0xffff;
      long count = h.getInt(107) & 0xffffffffL;
      double[] scale = {h.getDouble(131), h.getDouble(139), h.getDouble(147)};
      double[] origin = {h.getDouble(155), h.getDouble(163), h.getDouble(171)};
      ByteBuffer v = ByteBuffer.allocate((int) (offset - headerSize));
      ch.read(v, headerSize);
      return new Las(offset, recordLength, count, scale, origin, v.array(), vlrCount);
    }
  }

  /** Two passes over the block clouds: bounds of the kept points, then the merged LAS. */
  static long[] mergeLas(List<Path> runs, Similarity[] transforms, Owners owners, Path target) throws IOException {
    List<Las> headers = new ArrayList<>();
    for (Path run : runs) headers.add(header(run.resolve("07_results/dense.las")));
    int length = headers.get(0).recordLength();
    for (Las h : headers)
      if (h.recordLength() != length) throw new IOException("Block clouds have different point record layouts");
    double[] min = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, max = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    long[] kept = new long[runs.size()];
    for (int b = 0; b < runs.size(); b++) {
      int block = b;
      scan(runs.get(b).resolve("07_results/dense.las"), headers.get(b), (xyz, record) -> {
        double[] p = transforms[block].apply(xyz);
        if (owners.owner(p[0], p[1]) != block) return;
        kept[block]++;
        for (int k = 0; k < 3; k++) { min[k] = Math.min(min[k], p[k]); max[k] = Math.max(max[k], p[k]); }
      });
    }
    long total = 0;
    for (long k : kept) total += k;
    if (total == 0) throw new IOException("No points were kept from the blocks");
    if (total > 0xffffffffL) throw new IOException("Merged cloud exceeds the LAS 1.2 point count limit");
    double quantization = .0001;
    for (int k = 0; k < 3; k++) quantization = Math.max(quantization, (max[k] - min[k]) / (2.0 * Integer.MAX_VALUE - 2));
    double q = quantization;
    double[] offsets = new double[3];
    for (int k = 0; k < 3; k++) offsets[k] = min[k] + (max[k] - min[k]) / 2;
    Las first = headers.get(0);
    Path part = target.resolveSibling(target.getFileName() + ".part");
    try (OutputStream o = new BufferedOutputStream(Files.newOutputStream(part), 1 << 20)) {
      ByteBuffer h = ByteBuffer.allocate(227).order(ByteOrder.LITTLE_ENDIAN);
      h.put("LASF".getBytes(StandardCharsets.US_ASCII));
      h.position(24);
      h.put((byte) 1).put((byte) 2);
      byte[] system = new byte[32], software = new byte[32];
      byte[] s1 = "LAS.utils photogrammetry".getBytes(StandardCharsets.US_ASCII), s2 = ("POSE " + PdfReport.VERSION + " blocks").getBytes(StandardCharsets.US_ASCII);
      System.arraycopy(s1, 0, system, 0, Math.min(32, s1.length));
      System.arraycopy(s2, 0, software, 0, Math.min(32, s2.length));
      h.put(system).put(software);
      LocalDate now = LocalDate.now();
      h.putShort((short) now.getDayOfYear()).putShort((short) now.getYear());
      h.putShort((short) 227).putInt(227 + first.vlrs().length).putInt(first.vlrCount()).put((byte) 2)
          .putShort((short) length).putInt((int) total).putInt((int) total);
      for (int i = 0; i < 4; i++) h.putInt(0);
      for (int i = 0; i < 3; i++) h.putDouble(q);
      for (double v : offsets) h.putDouble(v);
      for (int k = 0; k < 3; k++) { h.putDouble(max[k]); h.putDouble(min[k]); }
      o.write(h.array());
      o.write(first.vlrs());
      ByteBuffer rec = ByteBuffer.allocate(length).order(ByteOrder.LITTLE_ENDIAN);
      for (int b = 0; b < runs.size(); b++) {
        int block = b;
        scan(runs.get(b).resolve("07_results/dense.las"), headers.get(b), (xyz, record) -> {
          double[] p = transforms[block].apply(xyz);
          if (owners.owner(p[0], p[1]) != block) return;
          rec.clear();
          rec.put(record);
          for (int k = 0; k < 3; k++) {
            long v = Math.round((p[k] - offsets[k]) / q);
            rec.putInt(4 * k, (int) v);
          }
          try {
            o.write(rec.array());
          } catch (IOException e) {
            throw new UncheckedIOException(e);
          }
        });
      }
    }
    Files.move(part, target, StandardCopyOption.REPLACE_EXISTING);
    return kept;
  }

  interface PointVisitor {
    void visit(double[] xyz, byte[] record);
  }

  private static void scan(Path file, Las h, PointVisitor visitor) throws IOException {
    try (FileChannel ch = FileChannel.open(file)) {
      int length = h.recordLength(), batch = 65536;
      ByteBuffer buffer = ByteBuffer.allocate(length * batch).order(ByteOrder.LITTLE_ENDIAN);
      long position = h.offset();
      byte[] record = new byte[length];
      double[] xyz = new double[3];
      for (long done = 0; done < h.count(); ) {
        int n = (int) Math.min(batch, h.count() - done);
        buffer.clear().limit(n * length);
        while (buffer.hasRemaining()) {
          int r = ch.read(buffer, position);
          if (r < 0) throw new EOFException(file.toString());
          position += r;
        }
        buffer.flip();
        for (int i = 0; i < n; i++) {
          buffer.get(record);
          ByteBuffer view = ByteBuffer.wrap(record).order(ByteOrder.LITTLE_ENDIAN);
          for (int k = 0; k < 3; k++) xyz[k] = view.getInt(4 * k) * h.scale()[k] + h.origin()[k];
          visitor.visit(xyz, record);
        }
        done += n;
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Status

  static int status(Path root) throws IOException {
    Path blocksDir = root.resolve("blocks");
    System.out.println("Block project " + root);
    Path calibration = blocksDir.resolve("calibration").resolve("camera.txt");
    System.out.println("  shared calibration: " + (Files.isRegularFile(calibration) ? "done" : "pending"));
    String plan = Files.readString(blocksDir.resolve("plan.json"));
    int count = plan.split("\"core_images\"", -1).length - 1;
    for (int id = 0; id < count; id++) {
      Path d = blockDir(blocksDir, id);
      String state = Files.isRegularFile(d.resolve("done.json")) ? "complete"
          : Files.isRegularFile(d.resolve("last-attempt.json")) ? "failed or interrupted (rerun the command to resume)"
          : "pending";
      System.out.println("  " + d.getFileName() + ": " + state);
    }
    Path merged = root.resolve("merged").resolve("dense.las");
    System.out.println("  merged cloud: " + (Files.isRegularFile(merged) ? merged : "pending"));
    return 0;
  }
}
