package sgm;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.*;
import java.time.*;
import java.time.format.*;
import java.util.*;
import java.util.List;
import javax.imageio.*;
import javax.imageio.stream.ImageInputStream;

/** Bounded report-resolution orthorectification and geometry-only coverage maps. */
final class ReportMaps {
  private record Source(Path file, String sha256) {}

  private record View(String name, Calibration calibration, Camera camera) {}

  private record Grid(int width, int height, float[] z, double x, double y, double cell) {}

  /** World rectangle drawn into a plot area of pw x ph pixels at (70, 80) of the canvas. */
  private record Bounds(double x, double y, double width, double height, int pw, int ph) {
    int canvasWidth() {
      return pw + 140;
    }

    int canvasHeight() {
      return ph + 170;
    }
  }

  static final int PLOT_WIDTH = 860, PLOT_HEIGHT = 700, PLOT_MIN = 240;

  /**
   * Plot size for a world width x height: as large as fits 860 x 700 pixels with the same aspect,
   * never thinner than 240 pixels (the world rectangle is widened instead, see fit()).
   */
  private static int[] plot(double width, double height) {
    double aspect = width / height;
    int pw, ph;
    if (aspect > PLOT_WIDTH / (double) PLOT_HEIGHT) {
      pw = PLOT_WIDTH;
      ph = (int) Math.round(PLOT_WIDTH / aspect);
    } else {
      ph = PLOT_HEIGHT;
      pw = (int) Math.round(PLOT_HEIGHT * aspect);
    }
    return new int[] {Math.max(PLOT_MIN, pw), Math.max(PLOT_MIN, ph)};
  }

  static Map<String, Object> create(
      Path run,
      Path out,
      Options options,
      Map<String, Object> dsm,
      Map<String, Object> coordinates,
      Map<String, Object> parameters)
      throws IOException {
    for (String name :
        List.of("orthomosaic.png", "orthomosaic.pgw", "overlap.png", "camera-map.png", "image-overlap.png", "maps.json"))
      Files.deleteIfExists(out.resolve(name));
    Options sourceOptions = options.copy();
    sourceOptions.orientationPixels = "source";
    OrientationIO.Model model;
    try {
      model = OrientationIO.readRun(run, sourceOptions);
    } catch (IOException | IllegalArgumentException missing) {
      return Json.object(
          "status",
          "unavailable",
          "reason",
          missing.getMessage(),
          "orthomosaic_note",
          "Camera orientations unavailable: " + missing.getMessage());
    }
    List<View> views = new ArrayList<>();
    for (var v : model.views.values())
      if (v.pose() != null && v.calibration() != null)
        views.add(new View(v.image(), v.calibration(), v.pose()));
    Map<String, Source> sources = sources(run);
    double scale = number(parameters.get("scale"), 1);
    if (!(scale > 0 && Double.isFinite(scale))) scale = 1;
    boolean north = Boolean.TRUE.equals(coordinates.get("georeferenced"));
    Grid grid = grid(out, dsm);
    Bounds bounds = bounds(views, grid, scale);
    Map<String, String> times = acquisition(run, sources);
    List<String> knownTimes = times.values().stream().sorted().toList();
    Map<String, Object> captures =
        Json.object(
            "capture_times_known",
            knownTimes.size(),
            "capture_first_known",
            knownTimes.isEmpty() ? null : knownTimes.getFirst(),
            "capture_last_known",
            knownTimes.isEmpty() ? null : knownTimes.getLast(),
            "capture_time_basis",
            knownTimes.isEmpty()
                ? "Unavailable"
                : knownTimes.getFirst().endsWith("Z")
                    ? "UTC (EXIF offset applied)"
                    : "EXIF local camera clock; timezone unavailable");
    boolean ordered =
        views.size() > 1
            && views.stream().allMatch(v -> times.containsKey(v.name))
            && new HashSet<>(times.values()).size() == times.size();
    List<View> flight = new ArrayList<>(views);
    if (ordered) flight.sort(Comparator.comparing(v -> times.get(v.name)));
    BufferedImage cameras =
        cameraMap(views, flight, ordered, bounds, grid, scale, north, sources.size());
    ImageIO.write(cameras, "png", out.resolve("camera-map.png").toFile());
    cameras.flush();
    String cameraNote =
        "Registered cameras: "
            + views.size()
            + " / "
            + sources.size()
            + " inventoried images. "
            + (ordered
                ? "Lines follow unique EXIF acquisition times. UTC offsets are applied when"
                    + " available; otherwise a common camera clock is assumed."
                : "Acquisition order unavailable or ambiguous; no flight lines inferred from"
                    + " filenames.")
            + " Arrows show the XY projection of viewing direction. Dashed footprints, where shown,"
            + " intersect the median DSM-height plane and are approximate. "
            + (north
                ? "Projected coordinates; north is +Y."
                : "Local XY projection; geographic north is unavailable.");
    if (grid == null) {
      Map<String, Object> partial =
          Json.object(
              "status",
              "partial",
              "registered_cameras",
              views.size(),
              "input_images",
              sources.size(),
              "camera_note",
              cameraNote,
              "orthomosaic_note",
              "Orthomosaic unavailable: no report DSM. "
                  + (dsm.get("reason") instanceof String why && !why.isBlank()
                      ? why
                      : "Use --report-local-dsm for a meaningful local XY/Z frame."),
              "overlap_note",
              "Coverage unavailable without a usable surface model.");
      partial.putAll(captures);
      Json.write(out.resolve("maps.json"), partial);
      return partial;
    }
    int n = grid.width * grid.height;
    int[] coverage = new int[n];
    float[] rank = new float[n];
    Arrays.fill(rank, Float.POSITIVE_INFINITY);
    BufferedImage ortho = new BufferedImage(grid.width, grid.height, BufferedImage.TYPE_INT_ARGB);
    int available = 0, unavailable = 0, painted = 0;
    try (WorkProgress work =
        WorkProgress.start("Report orthomosaic and geometric coverage", views.size(), "images")) {
      int completed = 0;
      int workers = Math.max(1, options.cores);
      // Source check and thumbnail decoding run a few images ahead on background threads; the
      // images are still painted one by one in their order, so the result does not change.
      java.util.concurrent.ExecutorService loader =
          java.util.concurrent.Executors.newFixedThreadPool(
              Math.min(4, workers),
              r -> {
                Thread t = new Thread(r, "report-thumbnail");
                t.setDaemon(true);
                return t;
              });
      List<java.util.concurrent.Future<BufferedImage>> photos = new ArrayList<>();
      int ahead = Math.min(4, workers);
      try {
        for (int v = 0; v < views.size(); v++) {
          while (photos.size() < views.size() && photos.size() <= v + ahead) {
            View next = views.get(photos.size());
            Source source = sources.get(next.name);
            photos.add(
                loader.submit(
                    () -> {
                      if (source == null || !Files.isRegularFile(source.file)) return null;
                      try {
                        if (!source.sha256.isBlank()
                            && ReuseCache.hash(source.file).equalsIgnoreCase(source.sha256))
                          return thumbnail(source.file, 1600);
                      } catch (IOException failure) {
                        System.err.println(
                            "Report source unavailable: " + next.name + "; " + failure.getMessage());
                      }
                      return null;
                    }));
          }
          View view = views.get(v);
          work.detail(view.name);
          BufferedImage photo;
          try {
            photo = photos.get(v).get();
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IOException("Report interrupted", e);
          } catch (java.util.concurrent.ExecutionException e) {
            throw new IOException(e.getCause());
          }
          photos.set(v, null);
          if (photo == null) unavailable++;
          else available++;
          Camera camera = view.camera;
          Camera k = view.calibration.camera(view.calibration.width, view.calibration.height);
          double[] center = camera.center();
          for (int j = 0; j < 3; j++) center[j] *= scale;
          int[] paintedRows = new int[grid.height];
          BufferedImage source = photo;
          double s = scale;
          // Cells are independent within one image: rows in parallel, same result.
          Parallel.forEach(grid.height, workers, y -> {
            for (int x = 0; x < grid.width; x++) {
              int i = y * grid.width + x;
              float z = grid.z[i];
              if (!Float.isFinite(z)) continue;
              double wx = grid.x + (x + .5) * grid.cell, wy = grid.y - (y + .5) * grid.cell;
              double[] q = camera.view(new double[] {wx / s, wy / s, z / s});
              if (!(q[2] > 0)) continue;
              Point2D.Double uv = project(k, view.calibration.distortion, q, view.calibration.width, view.calibration.height);
              if (!Double.isFinite(uv.x + uv.y)
                  || uv.x < 0
                  || uv.y < 0
                  || uv.x >= view.calibration.width - 1
                  || uv.y >= view.calibration.height - 1) continue;
              coverage[i]++;
              double score = Math.pow((uv.x - k.cx) / k.fx, 2) + Math.pow((uv.y - k.cy) / k.fy, 2);
              if (source == null || score >= rank[i] || !visible(grid, center, wx, wy, z)) continue;
              double px = (uv.x + .5) * source.getWidth() / view.calibration.width - .5;
              double py = (uv.y + .5) * source.getHeight() / view.calibration.height - .5;
              if (px < 0 || py < 0 || px >= source.getWidth() - 1 || py >= source.getHeight() - 1)
                continue;
              if (!Float.isFinite(rank[i])) paintedRows[y]++;
              ortho.setRGB(x, y, 0xff000000 | bilinear(source, px, py));
              rank[i] = (float) score;
            }
          });
          for (int count : paintedRows) painted += count;
          if (photo != null) photo.flush();
          work.update(++completed);
        }
      } finally {
        loader.shutdownNow();
      }
      work.done();
    }
    // A classic or mosaic orthomosaic of the run has no visibility holes; the overview then shows
    // that file (sampled at the report cells) instead of the occlusion-tested report render.
    String fromMosaic = null;
    Path results = run.resolve("07_results");
    Map<String, Object> full = ProcessingReport.map(results.resolve("orthomosaic.json"));
    if ("complete".equals(full.get("status"))
        && ("classic".equals(full.get("mode")) || "mosaic".equals(full.get("mode")))
        && full.get("extent") instanceof List<?> extent && extent.size() == 4
        && full.get("cell_size") instanceof Number fullCell
        && Files.isRegularFile(results.resolve("orthomosaic.tif"))) {
      double ex0 = ((Number) extent.get(0)).doubleValue(), eyTop = ((Number) extent.get(3)).doubleValue(), fc = fullCell.doubleValue();
      try (TiledGeoTiff.Reader reader = new TiledGeoTiff.Reader(results.resolve("orthomosaic.tif"))) {
        int sampled = 0;
        for (int y = 0; y < grid.height; y++)
          for (int x = 0; x < grid.width; x++) {
            double wx = grid.x + (x + .5) * grid.cell, wy = grid.y - (y + .5) * grid.cell;
            int value = reader.pixel((int) Math.floor((wx - ex0) / fc), (int) Math.floor((eyTop - wy) / fc));
            ortho.setRGB(x, y, (value >>> 24) == 0 ? 0 : value | 0xff000000);
            if ((value >>> 24) != 0) sampled++;
          }
        painted = sampled;
        fromMosaic = (String) full.get("mode");
      } catch (IOException unreadable) {
        System.err.println("Report orthomosaic: " + unreadable.getMessage() + "; using the report render");
      }
    }
    if (painted > 0) {
      ImageIO.write(ortho, "png", out.resolve("orthomosaic.png").toFile());
      Files.writeString(
          out.resolve("orthomosaic.pgw"),
          String.format(
              Locale.ROOT,
              "%.12f%n0%n0%n%.12f%n%.12f%n%.12f%n",
              grid.cell,
              -grid.cell,
              grid.x + grid.cell / 2,
              grid.y - grid.cell / 2));
    }
    ortho.flush();
    BufferedImage overlap = overlap(grid, coverage, views.size(), north);
    ImageIO.write(overlap, "png", out.resolve("overlap.png").toFile());
    overlap.flush();
    long overlapStarted = System.nanoTime();
    Object[] imageOverlap = imageOverlap(views, grid, bounds, scale, north, options.cores);
    ImageIO.write((BufferedImage) imageOverlap[0], "png", out.resolve("image-overlap.png").toFile());
    ((BufferedImage) imageOverlap[0]).flush();
    long[] overlapClasses = (long[]) imageOverlap[1];
    RunTelemetry.record("Report image overlap map", overlapStarted);
    Map<Integer, Long> histogram = new TreeMap<>();
    long supported = 0;
    for (int i = 0; i < n; i++)
      if (Float.isFinite(grid.z[i])) {
        histogram.merge(coverage[i], 1L, Long::sum);
        supported++;
      }
    String note =
        fromMosaic != null
            ? "Overview of the full-resolution " + fromMosaic + " orthomosaic (07_results/orthomosaic.tif) at report"
                + " resolution: " + grid.cell + " output units/pixel. See orthomosaic.json for its method."
            : painted > 0
            ? "Orthorectified original Brown-calibrated photographs onto the archived "
                + dsm.get("statistic")
                + "-cell DSM, with DSM ray-occlusion checks and nearest-center source selection."
                + " Report resolution: "
                + grid.cell
                + " output units/pixel. "
                + painted
                + " colored cells / "
                + supported
                + " surface cells; "
                + available
                + " verified source images, "
                + unavailable
                + " missing/changed/unreadable. Transparent cells are unavailable; no invented gap"
                + " filling. Vegetation/roof surface, not bare-earth terrain. No seam blending."
            : "Orthomosaic unavailable: no verified readable source image projects onto the report"
                + " surface.";
    Map<String, Object> result =
        Json.object(
            "status",
            painted > 0 ? "complete" : "partial",
            "registered_cameras",
            views.size(),
            "input_images",
            sources.size(),
            "orthomosaic_note",
            note,
            "camera_note",
            cameraNote,
            "image_overlap_note",
            "Number of photographs whose frame contains each ground point of the camera-map area"
                + " (report DSM height where known, else its median height; no occlusion test),"
                + " like the image overlap figure of other photogrammetry reports. Camera centres"
                + " are black dots.",
            "image_overlap_classes",
            overlapClasses,
            "overlap_note",
            "Geometric camera-frustum coverage at supported DSM cells; not reconstructed depth"
                + " support or confidence. DSM holes are unknown. Source-image readability does not"
                + " change geometric coverage.",
            "orthomosaic_width",
            grid.width,
            "orthomosaic_height",
            grid.height,
            "cell_size",
            grid.cell,
            "colored_cells",
            painted,
            "surface_cells",
            supported,
            "coverage_histogram",
            histogram,
            "verified_source_images",
            available,
            "unavailable_source_images",
            unavailable,
            "acquisition_order_established",
            ordered,
            "north_established",
            north,
            "coordinate_multiplier",
            scale);
    result.putAll(captures);
    Json.write(out.resolve("maps.json"), result);
    return result;
  }

  /**
   * Distorted pixel of camera-frame point q; NaN when its undistorted pixel lies more than a
   * quarter of the image outside the frame. The distortion polynomial folds back far outside the
   * image, which made cameras that cannot see a cell claim it near their image centre and paint
   * the wrong ground (shifted blocks in the overview; same guard as Orthomosaic.View.project).
   */
  private static Point2D.Double project(Camera k, double[] d, double[] q, int width, int height) {
    double pu = k.fx * q[0] / q[2] + k.cx, pv = k.fy * q[1] / q[2] + k.cy;
    if (pu < -.25 * width || pv < -.25 * height || pu > 1.25 * width || pv > 1.25 * height)
      return new Point2D.Double(Double.NaN, Double.NaN);
    org.opencv.core.Point p = SharedCalibration.distorted(k, d, new org.opencv.core.Point(pu, pv));
    return new Point2D.Double(p.x, p.y);
  }

  private static double number(Object value, double fallback) {
    if (value instanceof Number n) return n.doubleValue();
    try {
      return Double.parseDouble(String.valueOf(value));
    } catch (NumberFormatException missing) {
      return fallback;
    }
  }

  private static Grid grid(Path out, Map<String, Object> metadata) throws IOException {
    if (!"complete".equals(metadata.get("status")) || !Files.isRegularFile(out.resolve("dsm.tif")))
      return null;
    // Own reader: ImageIO returned 1.0 for every finite height (see DsmRaster.readTiff).
    DsmRaster.Heights dsm = DsmRaster.readTiff(out.resolve("dsm.tif"));
    int step = Math.max(1, (int) Math.ceil(Math.max(dsm.width(), dsm.height()) / 1024.)),
        offset = step / 2;
    int w = (dsm.width() - offset + step - 1) / step, h = (dsm.height() - offset + step - 1) / step;
    float[] z = new float[w * h];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
        z[y * w + x] = dsm.z()[(offset + y * step) * dsm.width() + offset + x * step];
    Object extent = metadata.get("extent");
    double[] box;
    if (extent instanceof double[] values) box = values;
    else box = JsonRead.numbers(extent);
    double cell = number(metadata.get("cell_size"), 1), shift = (offset + .5 - step * .5) * cell;
    return new Grid(w, h, z, box[0] + shift, box[3] - shift, cell * step);
  }

  /** Photograph file per image name (archived copy when the run has one). */
  static Map<String, Path> sourceFiles(Path run) throws IOException {
    Map<String, Path> files = new LinkedHashMap<>();
    for (var e : sources(run).entrySet()) files.put(e.getKey(), e.getValue().file);
    return files;
  }

  private static Map<String, Source> sources(Path run) throws IOException {
    Map<String, Source> sources = new LinkedHashMap<>();
    Path file = run.resolve("00_inputs/images.csv");
    if (!Files.isRegularFile(file)) return sources;
    try (BufferedReader in = Files.newBufferedReader(file)) {
      in.readLine();
      String line;
      while ((line = in.readLine()) != null) {
        List<String> fields = PoseIO.csv(line);
        if (fields.size() < 7) continue;
        Path path = Path.of(fields.get(2));
        if (fields.size() > 10 && !fields.get(10).isBlank()) {
          Path archived = run.resolve("00_inputs").resolve(fields.get(10)).normalize();
          if (archived.startsWith(run.resolve("00_inputs").normalize())
              && Files.isRegularFile(archived)) path = archived;
        }
        sources.put(fields.get(1), new Source(path, fields.get(4)));
      }
    }
    return sources;
  }

  private static Map<String, String> acquisition(Path run, Map<String, Source> sources)
      throws IOException {
    Map<String, String> result = new HashMap<>();
    List<Map<String, Object>> records = new ArrayList<>();
    Object archived = ProcessingReport.read(run.resolve("00_inputs/acquisition.json"));
    if (archived instanceof List<?> entries)
      for (Object item : entries) {
        Map<String, Object> entry = ProcessingReport.object(item);
        records.add(entry);
      }
    if (records.isEmpty())
      for (var entry : sources.entrySet())
        try {
          if (Files.isRegularFile(entry.getValue().file)
              && !entry.getValue().sha256.isBlank()
              && ReuseCache.hash(entry.getValue().file).equalsIgnoreCase(entry.getValue().sha256)) {
            Exif.Metadata metadata = Exif.read(entry.getValue().file);
            records.add(
                Json.object(
                    "filename",
                    entry.getKey(),
                    "capture_time_local",
                    metadata.captureTime,
                    "capture_offset",
                    metadata.captureOffset,
                    "camera_model",
                    metadata.make + "/" + metadata.model));
          }
        } catch (IOException ignored) {
        }
    Set<String> models = new HashSet<>();
    int offsetCount = 0, localCount = 0;
    DateTimeFormatter format =
        DateTimeFormatter.ofPattern("uuuu:MM:dd HH:mm:ss").withResolverStyle(ResolverStyle.STRICT);
    for (var record : records)
      try {
        LocalDateTime time =
            LocalDateTime.parse(String.valueOf(record.get("capture_time_local")), format);
        String offset = String.valueOf(record.getOrDefault("capture_offset", ""));
        String value;
        if (offset.matches("[+-]\\d{2}:\\d{2}")) {
          value = time.toInstant(ZoneOffset.of(offset)).toString();
          offsetCount++;
        } else {
          value = time.toString();
          localCount++;
          models.add(String.valueOf(record.getOrDefault("camera_model", "")));
        }
        result.put(String.valueOf(record.get("filename")), value);
      } catch (DateTimeException invalid) {
        /* No inferred timestamp. */
      }
    // Mixed local/UTC clocks and uncorrected clocks from different camera models have no safe
    // order.
    if (offsetCount > 0 && localCount > 0 || localCount > 0 && models.size() > 1) return Map.of();
    return result;
  }

  private static BufferedImage thumbnail(Path file, int cap) throws IOException {
    try (ImageInputStream input = ImageIO.createImageInputStream(file.toFile())) {
      Iterator<ImageReader> readers = ImageIO.getImageReaders(input);
      if (!readers.hasNext()) return null;
      ImageReader reader = readers.next();
      try {
        reader.setInput(input, true, true);
        int step =
            Math.max(
                1,
                (int) Math.ceil(Math.max(reader.getWidth(0), reader.getHeight(0)) / (double) cap));
        ImageReadParam param = reader.getDefaultReadParam();
        param.setSourceSubsampling(step, step, 0, 0);
        return reader.read(0, param);
      } finally {
        reader.dispose();
      }
    }
  }

  private static int bilinear(BufferedImage image, double x, double y) {
    int ix = (int) x, iy = (int) y;
    double fx = x - ix, fy = y - iy;
    int rgb = 0;
    int a = image.getRGB(ix, iy),
        b = image.getRGB(ix + 1, iy),
        c = image.getRGB(ix, iy + 1),
        d = image.getRGB(ix + 1, iy + 1);
    for (int shift : new int[] {16, 8, 0}) {
      double top = (1 - fx) * ((a >> shift) & 255) + fx * ((b >> shift) & 255),
          bottom = (1 - fx) * ((c >> shift) & 255) + fx * ((d >> shift) & 255);
      rgb |= ((int) Math.round((1 - fy) * top + fy * bottom)) << shift;
    }
    return rgb;
  }

  private static boolean visible(Grid g, double[] camera, double x, double y, double z) {
    int steps =
        Math.min(
            512, Math.max(2, (int) Math.ceil(Math.hypot(x - camera[0], y - camera[1]) / g.cell)));
    for (int k = 1; k < steps; k++) {
      double t = k / (double) steps;
      int xx = (int) Math.floor((camera[0] + (x - camera[0]) * t - g.x) / g.cell);
      int yy = (int) Math.floor((g.y - camera[1] - (y - camera[1]) * t) / g.cell);
      if (xx < 0 || yy < 0 || xx >= g.width || yy >= g.height) continue;
      float surface = g.z[yy * g.width + xx];
      double ray = camera[2] + (z - camera[2]) * t;
      if (Float.isFinite(surface) && surface > ray + 2 * g.cell) return false;
    }
    return true;
  }

  private static Bounds bounds(List<View> views, Grid grid, double scale) {
    double loX = grid == null ? Double.POSITIVE_INFINITY : grid.x,
        loY = grid == null ? Double.POSITIVE_INFINITY : grid.y - grid.height * grid.cell;
    double hiX = grid == null ? Double.NEGATIVE_INFINITY : grid.x + grid.width * grid.cell,
        hiY = grid == null ? Double.NEGATIVE_INFINITY : grid.y;
    for (View v : views) {
      double[] c = v.camera.center();
      loX = Math.min(loX, c[0] * scale);
      hiX = Math.max(hiX, c[0] * scale);
      loY = Math.min(loY, c[1] * scale);
      hiY = Math.max(hiY, c[1] * scale);
    }
    if (!Double.isFinite(loX + loY + hiX + hiY)) return new Bounds(0, 0, 1, 1, PLOT_MIN, PLOT_MIN);
    double width = Math.max(1e-6, hiX - loX),
        height = Math.max(1e-6, hiY - loY),
        border = .08 * Math.max(width, height);
    loX -= border;
    loY -= border;
    width += 2 * border;
    height += 2 * border;
    // The plot follows the area's shape (until 14 November 2026 a fixed 860 x 390 box, which
    // drew a north-south block as a thin column of cameras in a wide empty frame).
    int[] p = plot(width, height);
    if (width / height > p[0] / (double) p[1]) {
      double next = width * p[1] / p[0];
      loY -= (next - height) / 2;
      height = next;
    } else {
      double next = height * p[0] / p[1];
      loX -= (next - width) / 2;
      width = next;
    }
    return new Bounds(loX, loY, width, height, p[0], p[1]);
  }

  private static int px(double x, Bounds b) {
    return 70 + (int) ((x - b.x) / b.width * b.pw);
  }

  private static int py(double y, Bounds b) {
    return 80 + b.ph - (int) ((y - b.y) / b.height * b.ph);
  }

  private static Graphics2D graphics(BufferedImage image) {
    Graphics2D g = image.createGraphics();
    g.setColor(new Color(245, 248, 250));
    g.fillRect(0, 0, image.getWidth(), image.getHeight());
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 15));
    return g;
  }

  private static BufferedImage cameraMap(
      List<View> views,
      List<View> flight,
      boolean ordered,
      Bounds b,
      Grid grid,
      double scale,
      boolean north,
      int inputs) {
    BufferedImage image = new BufferedImage(b.canvasWidth(), b.canvasHeight(), BufferedImage.TYPE_INT_RGB);
    Graphics2D g = graphics(image);
    g.setColor(new Color(190, 202, 210));
    g.drawRect(70, 80, b.pw, b.ph);
    if (ordered) {
      g.setColor(new Color(85, 139, 167));
      for (int i = 1; i < flight.size(); i++) {
        double[] a = flight.get(i - 1).camera.center(), c = flight.get(i).camera.center();
        g.drawLine(
            px(a[0] * scale, b), py(a[1] * scale, b), px(c[0] * scale, b), py(c[1] * scale, b));
      }
    }
    float[] finite = grid == null ? new float[0] : grid.z.clone();
    Arrays.sort(finite);
    int end = 0;
    while (end < finite.length && Float.isFinite(finite[end])) end++;
    double plane = end == 0 ? Double.NaN : finite[end / 2];
    int id = 0;
    for (View view : views) {
      double[] c = view.camera.center();
      for (int j = 0; j < 3; j++) c[j] *= scale;
      int x = px(c[0], b), y = py(c[1], b);
      g.setColor(new Color(0, 105, 120));
      g.fillOval(x - 5, y - 5, 10, 10);
      if (views.size() <= 40) g.drawString(Integer.toString(id), x + 7, y - 6);
      id++;
      double dx = view.camera.rotation[6],
          dy = view.camera.rotation[7],
          length = Math.hypot(dx, dy);
      if (length > .01)
        g.drawLine(x, y, x + (int) (24 * dx / length), y - (int) (24 * dy / length));
      if (Double.isFinite(plane) && views.size() <= 25) footprint(g, view, c, plane, b, scale);
    }
    g.setColor(new Color(30, 55, 70));
    g.drawString(
        "Registered: "
            + views.size()
            + "   Unregistered/no pose: "
            + Math.max(0, inputs - views.size()),
        70,
        35);
    g.drawString(
        String.format(
            Locale.ROOT,
            "X %.2f to %.2f; Y %.2f to %.2f (output units)",
            b.x,
            b.x + b.width,
            b.y,
            b.y + b.height),
        70,
        b.ph + 130);
    compass(g, north, b.canvasWidth());
    scale(g, b.width / b.pw, 70, b.ph + 105);
    g.dispose();
    return image;
  }

  private static void footprint(
      Graphics2D g, View view, double[] center, double z, Bounds b, double scale) {
    Camera k = view.calibration.camera(view.calibration.width, view.calibration.height);
    int[] xs = new int[4], ys = new int[4];
    for (int i = 0; i < 4; i++) {
      double xd = ((i == 0 || i == 3 ? 0 : view.calibration.width - 1) - k.cx) / k.fx,
          yd = ((i < 2 ? 0 : view.calibration.height - 1) - k.cy) / k.fy,
          x = xd,
          y = yd;
      double[] d = view.calibration.distortion;
      for (int j = 0; j < 20; j++) {
        double r = x * x + y * y, a = 1 + d[0] * r + d[1] * r * r + d[4] * r * r * r;
        x = (xd - 2 * d[2] * x * y - d[3] * (r + 2 * x * x)) / a;
        y = (yd - d[2] * (r + 2 * y * y) - 2 * d[3] * x * y) / a;
      }
      double[] ray = Math3.mv(Math3.transpose(view.camera.rotation), new double[] {x, y, 1});
      double t = (z - center[2]) / ray[2];
      if (!(t > 0) || !Double.isFinite(t) || Math.abs(ray[2]) < .01) return;
      xs[i] = px(center[0] + t * ray[0], b);
      ys[i] = py(center[1] + t * ray[1], b);
    }
    Stroke old = g.getStroke();
    g.setStroke(new BasicStroke(1, 0, 0, 10, new float[] {5, 4}, 0));
    g.setColor(new Color(164, 184, 193));
    g.setClip(70, 80, b.pw, b.ph);
    g.drawPolygon(xs, ys, 4);
    g.setClip(null);
    g.setStroke(old);
  }

  private static BufferedImage overlap(Grid grid, int[] counts, int maximum, boolean north) {
    int[] p = plot(grid.width, grid.height);
    int canvasWidth = Math.max(p[0] + 140, 560);
    BufferedImage image = new BufferedImage(canvasWidth, p[1] + 170, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = graphics(image);
    BufferedImage raster = new BufferedImage(grid.width, grid.height, BufferedImage.TYPE_INT_RGB);
    for (int y = 0; y < grid.height; y++)
      for (int x = 0; x < grid.width; x++) {
        int i = y * grid.width + x;
        raster.setRGB(x, y, Float.isFinite(grid.z[i]) ? color(counts[i], maximum) : 0xe1e5e8);
      }
    double s = Math.min(p[0] / (double) grid.width, p[1] / (double) grid.height);
    int w = (int) (grid.width * s), h = (int) (grid.height * s);
    g.drawImage(raster, 70 + (p[0] - w) / 2, 80 + (p[1] - h) / 2, w, h, null);
    raster.flush();
    g.setColor(new Color(30, 55, 70));
    g.drawString("Camera-frustum coverage on DSM | gray: unknown surface", 70, 35);
    int entries = Math.min(maximum, 10) + 1, step = Math.min(65, (canvasWidth - 100) / entries);
    for (int i = 0; i < entries; i++) {
      int count = (int) Math.round(i * maximum / (double) Math.max(1, entries - 1));
      int x = 70 + i * step;
      g.setColor(new Color(color(count, maximum)));
      g.fillRect(x, p[1] + 100, step - 20, 15);
      g.setColor(Color.DARK_GRAY);
      g.drawString(Integer.toString(count), x + 4, p[1] + 137);
    }
    compass(g, north, canvasWidth);
    g.dispose();
    return image;
  }

  // Image overlap classes 1..9 and >9, red (one image) to blue (more than nine).
  private static final int[] OVERLAP_COLOURS = {
    0xa50026, 0xd73027, 0xf46d43, 0xfdae61, 0xfee08b, 0xd9ef8b, 0xa6d96a, 0x66bd63, 0x1a9850, 0x2166ac
  };

  /**
   * Metashape-style "camera locations and image overlap": over the whole camera-map area, the
   * number of photographs whose frame contains each ground point (DSM height where the report DSM
   * has one, else its median height; no occlusion test), with the camera centres on top.
   * Returns {image, histogram (index 0 = no image, 1..9, 10 = more than 9)}.
   */
  private static Object[] imageOverlap(
      List<View> views, Grid grid, Bounds b, double scale, boolean north, int cores) {
    int rw = Math.max(1, b.pw / 2), rh = Math.max(1, b.ph / 2);
    float[] finite = grid.z.clone();
    Arrays.sort(finite);
    int end = 0;
    while (end < finite.length && Float.isFinite(finite[end])) end++;
    double plane = end == 0 ? 0 : finite[end / 2];
    int[] counts = new int[rw * rh];
    Camera[] ks = new Camera[views.size()];
    for (int v = 0; v < ks.length; v++)
      ks[v] = views.get(v).calibration.camera(views.get(v).calibration.width, views.get(v).calibration.height);
    Parallel.forEach(rh, Math.max(1, cores), j -> {
      double wy = b.y + b.height - (j + .5) * b.height / rh;
      for (int i = 0; i < rw; i++) {
        double wx = b.x + (i + .5) * b.width / rw;
        int gx = (int) Math.floor((wx - grid.x) / grid.cell), gy = (int) Math.floor((grid.y - wy) / grid.cell);
        double z = plane;
        if (gx >= 0 && gy >= 0 && gx < grid.width && gy < grid.height && Float.isFinite(grid.z[gy * grid.width + gx]))
          z = grid.z[gy * grid.width + gx];
        int n = 0;
        for (int vi = 0; vi < ks.length; vi++) {
          View v = views.get(vi);
          double[] q = v.camera.view(new double[] {wx / scale, wy / scale, z / scale});
          if (!(q[2] > 0)) continue;
          Camera k = ks[vi];
          Point2D.Double uv = project(k, v.calibration.distortion, q, v.calibration.width, v.calibration.height);
          if (uv.x >= 0 && uv.y >= 0 && uv.x <= v.calibration.width - 1 && uv.y <= v.calibration.height - 1) n++;
        }
        counts[j * rw + i] = n;
      }
    });
    long[] histogram = new long[11];
    BufferedImage raster = new BufferedImage(rw, rh, BufferedImage.TYPE_INT_RGB);
    for (int j = 0; j < rh; j++)
      for (int i = 0; i < rw; i++) {
        int n = counts[j * rw + i];
        histogram[Math.min(10, n)]++;
        raster.setRGB(i, j, n == 0 ? 0xffffff : OVERLAP_COLOURS[Math.min(10, n) - 1]);
      }
    int canvasWidth = Math.max(b.canvasWidth(), 560);
    BufferedImage image = new BufferedImage(canvasWidth, b.canvasHeight(), BufferedImage.TYPE_INT_RGB);
    Graphics2D g = graphics(image);
    g.drawImage(raster, 70, 80, b.pw, b.ph, null);
    raster.flush();
    g.setColor(new Color(190, 202, 210));
    g.drawRect(70, 80, b.pw, b.ph);
    g.setColor(Color.BLACK);
    int dot = views.size() > 300 ? 2 : 3;
    for (View v : views) {
      double[] c = v.camera.center();
      g.fillOval(px(c[0] * scale, b) - dot, py(c[1] * scale, b) - dot, 2 * dot, 2 * dot);
    }
    g.setColor(new Color(30, 55, 70));
    g.drawString("Number of overlapping images | black dots: camera locations", 70, 35);
    int step = Math.min(60, (canvasWidth - 100) / 10);
    for (int c = 0; c < 10; c++) {
      int x = 70 + c * step;
      g.setColor(new Color(OVERLAP_COLOURS[c]));
      g.fillRect(x, b.ph + 100, step - 16, 15);
      g.setColor(Color.DARK_GRAY);
      g.drawString(c < 9 ? Integer.toString(c + 1) : ">9", x + 4, b.ph + 137);
    }
    compass(g, north, canvasWidth);
    scale(g, b.width / b.pw, canvasWidth - 270, 68);
    g.dispose();
    return new Object[] {image, histogram};
  }

  private static int color(int count, int max) {
    return count == 0
        ? 0xffffff
        : Color.HSBtoRGB((float) (.65 * (count - 1) / Math.max(1, max - 1)), .75f, .88f) & 0xffffff;
  }

  private static void compass(Graphics2D g, boolean north, int canvasWidth) {
    int x = canvasWidth - 40;
    g.setColor(new Color(30, 55, 70));
    g.drawLine(x, 100, x, 60);
    g.drawLine(x, 60, x - 7, 72);
    g.drawLine(x, 60, x + 7, 72);
    g.drawString(north ? "N" : "+Y", x - 10, 48);
  }

  private static void scale(Graphics2D g, double unitsPerPixel, int x, int y) {
    double value = unitsPerPixel * 160,
        power = Math.pow(10, Math.floor(Math.log10(value))),
        nice = Math.floor(value / power) * power;
    int width = (int) (nice / unitsPerPixel);
    g.setColor(new Color(30, 55, 70));
    g.drawLine(x, y, x + width, y);
    g.drawLine(x, y - 4, x, y + 4);
    g.drawLine(x + width, y - 4, x + width, y + 4);
    g.drawString(String.format(Locale.ROOT, "%.3g units", nice), x, y - 8);
  }
}
