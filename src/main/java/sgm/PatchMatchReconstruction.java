package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/** Adapts per-reference depth/normal maps to the common pair-provenance fusion stream. */
final class PatchMatchReconstruction {
  private final Options options;
  private final List<ImageFrame> frames;
  private final List<FeatureTracks.Track> tracks;
  private final DepthFusion fusion;
  private final ProjectArtifacts artifacts;
  private final List<int[]> sources = new ArrayList<>();
  private final Map<Long, Integer> pairIds = new LinkedHashMap<>();
  private final Map<Integer, Path> maps = new HashMap<>();
  private long reciprocalRejected, matchingRejected, angleRejected, emitted;
  private long unavailableReference, geometricRejected, depthLimitRejected;
  private long ambiguityRejected;

  PatchMatchReconstruction(
      Options o,
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      DepthFusion fusion,
      ProjectArtifacts artifacts) {
    options = o;
    this.frames = frames;
    this.tracks = tracks;
    this.fusion = fusion;
    this.artifacts = artifacts;
  }

  long run(List<FeatureTracks.Pair> pairs, DataOutputStream out) throws IOException {
    for (var f : frames) {
      List<Integer> choices = new ArrayList<>();
      if (!f.camera.registered) {
        sources.add(new int[0]);
        continue;
      }
      for (var p : pairs) {
        int other = p.a == f.id ? p.b : p.b == f.id ? p.a : -1;
        if (other >= 0
            && other != f.id
            && frames.get(other).camera.registered
            && !choices.contains(other)
            && Math3.norm(Math3.sub(f.camera.center(), frames.get(other).camera.center())) > 1e-8)
          choices.add(other);
      }
      choices.sort(
          Comparator.comparingDouble(
              i -> Math3.norm(Math3.sub(f.camera.center(), frames.get(i).camera.center()))));
      sources.add(
          choices.stream().limit(options.patchmatchViews).mapToInt(Integer::intValue).toArray());
      for (int other : sources.get(f.id)) {
        int source = fusion.addPair(f.id, other);
        pairIds.put(key(f.id, other), source);
        if (artifacts != null) artifacts.patchmatchPair(source, f, frames.get(other));
      }
    }
    estimateMaps();
    // One owner and one source map resident at a time; no all-camera depth-map heap.
    for (var f : frames) {
      if (!maps.containsKey(f.id)) continue;
      PatchMatch.Map map = read(maps.get(f.id));
      for (int i = 0; i < map.width() * map.height(); i++) {
        if (map.view(i) == -2) geometricRejected++;
        else if (Float.isFinite(map.photo(i)) && map.photo(i) < options.correlation)
          matchingRejected++;
        else if (map.view(i) < 1 || !Float.isFinite(map.depth(i))) unavailableReference++;
        else if (map.depth(i) > options.maxDepth) depthLimitRejected++;
      }
      f.ensureRasters();
      byte[] colors = new byte[f.width * f.height * 3];
      f.color.get(0, 0, colors);
      int[] selected = sources.get(f.id);
      for (int slot = 0; slot < selected.length; slot++) {
        int image = selected[slot];
        if (!maps.containsKey(image)) continue;
        var other = frames.get(image);
        var check = read(maps.get(image));
        int pair = pairIds.get(key(f.id, image));
        try (WorkProgress work =
            WorkProgress.start(
                "PatchMatch reciprocal evidence " + f.id + "-" + image, f.height, "rows")) {
          for (int y = 2; y < f.height - 2; y++) {
            for (int x = 2; x < f.width - 2; x++) {
              int i = y * f.width + x;
              if (map.view(i) != slot + 1) continue;
              if (!valid(map, i)) {
                continue;
              }
              double[] world = world(f.camera, x, y, map.depth(i));
              double angle =
                  DenseReconstruction.triangulationAngle(
                      world, f.camera.center(), other.camera.center());
              if (angle < options.minAngle) {
                angleRejected++;
                continue;
              }
              double lr = reciprocal(f.camera, other.camera, world, check, options);
              if (!Double.isFinite(lr)) {
                reciprocalRejected++;
                continue;
              }
              double[] uv = other.camera.project(world);
              int sourcePixel = (int) Math.round(uv[1]) * check.width() + (int) Math.round(uv[0]);
              double margin = Math.min(map.margin(i), check.margin(sourcePixel));
              double sharpness = Math.min(map.sharpness(i), check.sharpness(sourcePixel));
              double photo = Math.min(map.photo(i), check.photo(sourcePixel));
              if (!(margin > 0 && sharpness > 0)) {
                ambiguityRejected++;
                continue;
              }
              double matching = PointConfidence.score(photo, margin, sharpness, lr);
              fusion.add(pair, world);
              if ((x - 2) % options.step != 0 || (y - 2) % options.step != 0) continue;
              out.writeDouble(world[0]);
              out.writeDouble(world[1]);
              out.writeDouble(world[2]);
              out.writeByte(colors[i * 3 + 2] & 255);
              out.writeByte(colors[i * 3 + 1] & 255);
              out.writeByte(colors[i * 3] & 255);
              out.writeInt(pair);
              out.writeInt(i);
              for (double value : new double[] {matching, margin, sharpness, photo, lr})
                out.writeFloat((float) value);
              emitted++;
            }
            if ((y & 31) == 0) work.update(y);
          }
          work.done();
        }
      }
    }
    if (!options.diagnostics || options.storage.equals("minimal"))
      for (Path map : maps.values()) Files.deleteIfExists(map);
    Json.write(
        options.output.resolve("05_stereo/patchmatch.json"),
        Json.object(
            "method",
            "patchmatch",
            "experimental",
            true,
            "confidence_model",
            PointConfidence.MODEL,
            "sampled_candidates",
            emitted,
            "photometric_rejected",
            matchingRejected,
            "ambiguous_producing_pair_pixels",
            ambiguityRejected,
            "pair_confidence_rule",
            "weaker photometry, margin and curvature from both producing maps; unavailable or"
                + " nonpositive uniqueness cannot corroborate another point",
            "unavailable_reference_pixels",
            unavailableReference,
            "geometric_contradiction_reference_pixels",
            geometricRejected,
            "depth_limit_reference_pixels",
            depthLimitRejected,
            "reciprocal_rejected",
            reciprocalRejected,
            "angle_rejected",
            angleRejected,
            "cache",
            "per-scale depth/plane hypotheses; complete image bytes, masks, relative camera blocks,"
                + " seed map, method and compute settings in key"));
    return emitted;
  }

  private boolean valid(PatchMatch.Map map, int i) {
    return Float.isFinite(map.depth(i))
        && map.depth(i) > 0
        && map.depth(i) <= options.maxDepth
        && map.view(i) > 0
        && map.photo(i) >= options.correlation;
  }

  /** Synchronous rounds: every geometric fit reads the same frozen source-map generation. */
  private void estimateMaps() throws IOException {
    long pixels = 0;
    for (ImageFrame f : frames)
      if (f.camera.registered && sources.get(f.id).length > 0)
        pixels = Math.addExact(pixels, (long) f.width * f.height);
    // Two frozen generations of float[8] maps and float[1] trusted-depth sidecars.
    // Final output/depth-evidence storage is additional and is budgeted by the normal pipeline.
    Files.createDirectories(options.output.resolve("05_stereo"));
    long scratch = Math.addExact(Math.multiplyExact(pixels, 72L), 16L << 20);
    long available = Files.getFileStore(options.output).getUsableSpace();
    if (available > 0 && scratch > available)
      throw new IOException(
          "PatchMatch synchronous map generations need up to "
              + scratch
              + " bytes of scratch storage; available "
              + available);
    Map<Integer, double[]> ranges = new HashMap<>();
    Map<Integer, List<Object>> reports = new HashMap<>();
    for (ImageFrame f : frames)
      if (f.camera.registered && sources.get(f.id).length > 0) {
        ranges.put(f.id, bounds(f));
        reports.put(f.id, new ArrayList<>());
      }
    Map<Integer, Path> generation = new HashMap<>();
    for (int level = options.patchmatchLevels - 1; level >= 0; level--)
      for (int pass = 0; pass < 3; pass++) {
        Map<Integer, Path> next = new HashMap<>();
        String phase = pass == 0 ? "photometric" : "geometric-" + pass;
        try (WorkProgress work =
            WorkProgress.start(
                "PatchMatch level " + level + " " + phase, ranges.size(), "references")) {
          int completed = 0;
          for (ImageFrame f : frames) {
            if (!ranges.containsKey(f.id)) continue;
            work.detail("reference " + f.id);
            long started = System.nanoTime();
            PatchMatch.Scene scene = scene(f, sources.get(f.id), 1 << level, ranges.get(f.id));
            if (pass > 0) scene = geometry(scene, sources.get(f.id), generation);
            PatchMatch.Map previous =
                generation.containsKey(f.id) ? read(generation.get(f.id)) : null;
            PatchMatch.Map result = ReuseCache.patchmatch(options, scene, previous);
            Path directory =
                options.output.resolve(
                    String.format(Locale.ROOT, "05_stereo/reference_%04d", f.id));
            Files.createDirectories(directory);
            Path file =
                directory.resolve(
                    level == 0 && pass == 2
                        ? "depth-normal-evidence.bin"
                        : "level-" + level + "-pass-" + pass + ".bin");
            try (var stream =
                new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file)))) {
              stream.writeInt(result.width());
              stream.writeInt(result.height());
              for (float value : result.data()) stream.writeFloat(value);
            }
            if (level != 0 || pass != 2) {
              try (var trusted =
                  new DataOutputStream(
                      new BufferedOutputStream(Files.newOutputStream(trusted(file))))) {
                trusted.writeInt(result.width());
                trusted.writeInt(result.height());
                for (int i = 0; i < result.width() * result.height(); i++)
                  trusted.writeFloat(
                      valid(result, i) && result.margin(i) >= .05 && result.sharpness(i) >= .05
                          ? result.depth(i)
                          : 0);
              }
            }
            next.put(f.id, file);
            reports
                .get(f.id)
                .add(
                    Json.object(
                        "level",
                        level,
                        "phase",
                        phase,
                        "width",
                        scene.width(),
                        "height",
                        scene.height(),
                        "effective_sweeps",
                        PatchMatch.sweeps(scene, options),
                        "seconds",
                        (System.nanoTime() - started) / 1e9,
                        "backend",
                        result.backend()));
            work.update(++completed);
            System.out.printf(
                Locale.ROOT,
                "PatchMatch image %d level %d %s: %dx%d, %.2f s%n",
                f.id,
                level,
                phase,
                scene.width(),
                scene.height(),
                (System.nanoTime() - started) / 1e9);
          }
          work.done();
        }
        // These paths were created above inside this run; no recursive or computed-tree deletion.
        for (Path old : generation.values()) {
          Files.deleteIfExists(old);
          Files.deleteIfExists(trusted(old));
        }
        generation = next;
      }
    maps.putAll(generation);
    for (ImageFrame f : frames)
      if (ranges.containsKey(f.id)) {
        double[] range = ranges.get(f.id);
        Json.write(
            maps.get(f.id).resolveSibling("patchmatch.json"),
            Json.object(
                "method",
                "patchmatch",
                "experimental",
                true,
                "estimator_model",
                PatchMatch.MODEL,
                "reference_image",
                f.id,
                "source_images",
                sources.get(f.id),
                "camera_z_minimum",
                range[0],
                "camera_z_maximum",
                range[1],
                "depth_bounds_source",
                options.patchmatchMinDepth > 0
                    ? "explicit"
                    : "aligned sparse points; padded 1st/99th percentiles",
                "requested_minimum_sweeps",
                options.patchmatchIterations,
                "geometric_rounds_per_level",
                2,
                "levels",
                reports.get(f.id),
                "invalid_match",
                "NaN depth and source_slot<1 (-2 denotes geometric contradiction); infinite cost"
                    + " distinguishes missing from poor photometry",
                "binary_format",
                "Big endian int32 width,height then pixel-major float32[8]:"
                    + " camera-Z,nx,ny,nz,1-ZNCC,best_source_slot,margin,sharpness",
                "confidence_competitors",
                "valid alternatives at 2/4/8/16-pixel depth offsets and broad inverse-depth probes;"
                    + " invalid competitors never improve confidence",
                "geometric_evidence",
                "coherent valid source depths; two free-space contradictions invalidate a"
                    + " hypothesis; foreground occlusion and unavailable depth are neutral; source"
                    + " maps are frozen per round",
                "filtering",
                "correlation, LR, minimum angle, depth uncertainty, fusion and confidence controls"
                    + " apply; SGM speckle/median controls do not",
                "view_selection",
                "nearest noncoincident cameras on selected pair graph, bounded source count",
                "fallback",
                "explicit CPU reference only with --compute cpu; GPU failures abort; never"
                    + " substitutes SGM"));
      }
  }

  private PatchMatch.Scene geometry(
      PatchMatch.Scene scene, int[] selected, Map<Integer, Path> input) throws IOException {
    float[] depths = new float[scene.pixels().length];
    float[] cameras = scene.cameras();
    for (int slot = 0; slot < selected.length; slot++) {
      Path file = input.get(selected[slot]);
      if (file == null) continue;
      int c = (slot + 1) * 20, width = (int) cameras[c + 16], height = (int) cameras[c + 17];
      int offset = (int) cameras[c + 18] * 65536 + (int) cameras[c + 19];
      try (var in =
          new DataInputStream(new BufferedInputStream(Files.newInputStream(trusted(file))))) {
        if (in.readInt() != width || in.readInt() != height)
          throw new IOException("Geometric source-map resolution mismatch");
        for (int i = 0; i < width * height; i++) depths[offset + i] = in.readFloat();
      }
    }
    return new PatchMatch.Scene(
        scene.width(),
        scene.height(),
        scene.pixels(),
        scene.masks(),
        cameras,
        scene.views(),
        scene.minimum(),
        scene.maximum(),
        depths,
        (float) options.fusionRelative,
        (float) options.patchmatchEdgeSigma);
  }

  private static long key(int a, int b) {
    return ((long) a << 32) | (b & 0xffffffffL);
  }

  private static Path trusted(Path file) {
    return file.resolveSibling(file.getFileName() + ".trusted");
  }

  static double[] world(Camera c, double x, double y, double z) {
    return Math3.mv(
        Math3.transpose(c.rotation),
        Math3.sub(new double[] {(x - c.cx) / c.fx * z, (y - c.cy) / c.fy * z, z}, c.translation));
  }

  static double reciprocal(
      Camera reference, Camera source, double[] world, PatchMatch.Map map, Options o) {
    double[] uv = source.project(world), v = source.view(world);
    if (uv == null
        || uv[0] < 2
        || uv[1] < 2
        || uv[0] >= map.width() - 2
        || uv[1] >= map.height() - 2) return Double.NaN;
    int x = (int) Math.round(uv[0]), y = (int) Math.round(uv[1]), i = y * map.width() + x;
    if (map.view(i) < 1 || map.photo(i) < o.correlation || !(map.depth(i) > 0)) return Double.NaN;
    float[] d = map.data();
    int at = i * 8;
    double plane =
        map.depth(i)
            * (d[at + 1] * (x - source.cx) / source.fx
                + d[at + 2] * (y - source.cy) / source.fy
                + d[at + 3]);
    double denominator =
        d[at + 1] * (uv[0] - source.cx) / source.fx
            + d[at + 2] * (uv[1] - source.cy) / source.fy
            + d[at + 3];
    double z = plane / denominator;
    if (!(denominator > .05)
        || !Double.isFinite(z)
        || z <= 0
        || Math.abs(z - v[2]) > Math.max(.01, 3 * o.fusionRelative) * Math.min(z, v[2]))
      return Double.NaN;
    double[] back = reference.project(world(source, uv[0], uv[1], z)),
        original = reference.project(world);
    if (back == null || original == null) return Double.NaN;
    double error = Math.hypot(back[0] - original[0], back[1] - original[1]);
    return error <= o.lrThreshold ? error : Double.NaN;
  }

  private double[] bounds(ImageFrame f) {
    if (options.patchmatchMinDepth > 0)
      return new double[] {options.patchmatchMinDepth, options.patchmatchMaxDepth};
    List<Double> z = new ArrayList<>();
    for (var t : tracks)
      if (t.point != null) {
        double[] p = f.camera.project(t.point);
        double d = f.camera.view(t.point)[2];
        if (p != null && d > 0 && p[0] >= 0 && p[1] >= 0 && p[0] < f.width && p[1] < f.height)
          z.add(d);
      }
    if (z.size() < 12)
      throw new IllegalArgumentException(
          "PatchMatch needs at least 12 visible sparse points for image "
              + f.id
              + " or explicit --patchmatch-min-depth and --patchmatch-max-depth (camera-Z,"
              + " reconstruction units)");
    Collections.sort(z);
    return new double[] {
      Math.max(1e-6, z.get((z.size() - 1) / 100) * .75), z.get((z.size() - 1) * 99 / 100) * 1.25
    };
  }

  PatchMatch.Scene scene(ImageFrame reference, int[] selected, int factor, double[] bounds)
      throws IOException {
    List<ImageFrame> views = new ArrayList<>();
    views.add(reference);
    for (int id : selected) views.add(frames.get(id));
    long total = 0;
    for (var f : views)
      total += Math.max(1, f.width / factor) * (long) Math.max(1, f.height / factor);
    long n = (long) Math.max(1, reference.width / factor) * Math.max(1, reference.height / factor);
    long host = 4 * total + 160 * n;
    if (total > Integer.MAX_VALUE
        || host
            > Math.min(
                options.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65)))
      throw new IllegalArgumentException(
          "PatchMatch image/plane working set exceeds --memory-mb/heap; reduce views/resolution");
    byte[] pixels = new byte[(int) total], masks = new byte[(int) total];
    float[] cameras = new float[views.size() * 20];
    int offset = 0;
    for (int i = 0; i < views.size(); i++) {
      var f = views.get(i);
      f.ensureRasters();
      int w = Math.max(1, f.width / factor), h = Math.max(1, f.height / factor);
      Mat gray = new Mat(), mask = new Mat();
      try {
        Imgproc.resize(f.gray, gray, new Size(w, h), 0, 0, Imgproc.INTER_AREA);
        Imgproc.resize(f.validMask, mask, new Size(w, h), 0, 0, Imgproc.INTER_NEAREST);
        byte[] a = new byte[w * h], b = new byte[w * h];
        gray.get(0, 0, a);
        mask.get(0, 0, b);
        System.arraycopy(a, 0, pixels, offset, a.length);
        System.arraycopy(b, 0, masks, offset, b.length);
      } finally {
        gray.release();
        mask.release();
      }
      double[] r = Math3.mul(f.camera.rotation, Math3.transpose(reference.camera.rotation));
      double[] t = Math3.sub(f.camera.translation, Math3.mv(r, reference.camera.translation));
      int at = i * 20;
      for (int k = 0; k < 9; k++) cameras[at + k] = (float) r[k];
      for (int k = 0; k < 3; k++) cameras[at + 9 + k] = (float) t[k];
      cameras[at + 12] = (float) (f.camera.fx * w / f.width);
      cameras[at + 13] = (float) (f.camera.fy * h / f.height);
      cameras[at + 14] = (float) ((f.camera.cx + .5) * w / f.width - .5);
      cameras[at + 15] = (float) ((f.camera.cy + .5) * h / f.height - .5);
      cameras[at + 16] = w;
      cameras[at + 17] = h;
      cameras[at + 18] = offset / 65536;
      cameras[at + 19] = offset % 65536;
      offset += w * h;
    }
    return new PatchMatch.Scene(
        Math.max(1, reference.width / factor),
        Math.max(1, reference.height / factor),
        pixels,
        masks,
        cameras,
        views.size(),
        (float) bounds[0],
        (float) bounds[1],
        null,
        (float) options.fusionRelative,
        (float) options.patchmatchEdgeSigma);
  }

  private static PatchMatch.Map read(Path file) throws IOException {
    try (var in = new DataInputStream(new BufferedInputStream(Files.newInputStream(file)))) {
      int w = in.readInt(), h = in.readInt();
      float[] data = new float[Math.multiplyExact(Math.multiplyExact(w, h), 8)];
      for (int i = 0; i < data.length; i++) data[i] = in.readFloat();
      return new PatchMatch.Map(w, h, data, null);
    }
  }
}
