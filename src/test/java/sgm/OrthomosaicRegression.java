package sgm;

import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.io.RandomAccessFile;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;

/**
 * Full-resolution true orthomosaic (Orthomosaic, TiledGeoTiff; ORTHOMOSAIC.md) and automatic
 * check points (GroundControl.holdOut), on synthetic data with known answers.
 *
 * <p>Scene: flat ground at height 0 with a smooth colour pattern, and a box of 6 x 6 x 8 m with a
 * different roof pattern and magenta walls. Nine nadir cameras 50 m up (two with lens distortion)
 * are rendered by ray casting; the cloud is the ground and roof sampled every 0.2 m, without
 * ground points in a strip beside the box.
 *
 * <ol>
 *   <li>surface: highest point per cell, gap continued from the lower side, bilinear on continuous
 *       surface and nearest at the roof step, line-of-sight test;
 *   <li>mosaic of 2,500 x 2,300 pixels (four blocks): every ground pixel shows the ground pattern
 *       at its own position and every roof pixel the roof pattern at its position (the roof does
 *       not lean), no wall colour anywhere away from the outline, same tiles on 1 and 8 threads;
 *   <li>with only the two western cameras the ground behind the box is transparent (seen by no
 *       camera); without the visibility test the same pixels show roof and wall (the test has
 *       teeth);
 *   <li>the GeoTIFF read back by an independent decoder equals the tiles; geo tags; the BigTIFF
 *       variant holds the same tiles; photographs from files, whole and by window;
 *   <li>automatic check points: which points are held back, and the refusals.
 * </ol>
 */
final class OrthomosaicRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    System.out.println("PASS orthomosaic and automatic check points: " + run());
  }

  static final double BX0 = 30, BX1 = 36, BY0 = 24, BY1 = 30, ROOF = 8;
  static final int SIZE = 400;
  static final int WALL = 0xff00ff;

  static int ground(double x, double y) {
    return rgb(128 + 100 * Math.sin(x / 3), 128 + 100 * Math.sin(y / 4), 128 + 100 * Math.sin((x + y) / 5));
  }

  static int roof(double x, double y) {
    return rgb(128 - 100 * Math.sin(x / 3), 128 - 100 * Math.sin(y / 4), 60 + 40 * Math.sin((x - y) / 5));
  }

  static int rgb(double r, double g, double b) {
    return (int) Math.round(r) << 16 | (int) Math.round(g) << 8 | (int) Math.round(b);
  }

  static boolean close(int a, int b, int tolerance) {
    for (int shift = 0; shift <= 16; shift += 8) if (Math.abs(((a >> shift) & 255) - ((b >> shift) & 255)) > tolerance) return false;
    return true;
  }

  /** Colour seen along the ray from c in direction d (descending). */
  static int cast(double[] c, double[] d) {
    double best = Double.POSITIVE_INFINITY;
    int colour = 0;
    double t = (ROOF - c[2]) / d[2];
    double x = c[0] + t * d[0], y = c[1] + t * d[1];
    if (t > 0 && x >= BX0 && x <= BX1 && y >= BY0 && y <= BY1) {
      best = t;
      colour = roof(x, y);
    }
    for (int side = 0; side < 4; side++) {
      double plane = side == 0 ? BX0 : side == 1 ? BX1 : side == 2 ? BY0 : BY1;
      t = side < 2 ? (plane - c[0]) / d[0] : (plane - c[1]) / d[1];
      if (!(t > 0) || t >= best) continue;
      double px = c[0] + t * d[0], py = c[1] + t * d[1], pz = c[2] + t * d[2];
      boolean on = pz >= 0 && pz <= ROOF && (side < 2 ? py >= BY0 && py <= BY1 : px >= BX0 && px <= BX1);
      if (on) {
        best = t;
        colour = WALL;
      }
    }
    t = -c[2] / d[2];
    if (t > 0 && t < best) colour = ground(c[0] + t * d[0], c[1] + t * d[1]);
    return colour;
  }

  static int run() throws Exception {
    checks = 0;
    // ---- Cameras and their photographs.
    List<Orthomosaic.View> views = new ArrayList<>();
    List<int[]> images = new ArrayList<>();
    for (int j = 0; j < 3; j++)
      for (int i = 0; i < 3; i++) {
        double[] centre = {10 + 15 * i, 5 + 15 * j, 50};
        Camera pose = new Camera(400, 400, 199.5, 199.5);
        pose.rotation = new double[] {1, 0, 0, 0, -1, 0, 0, 0, -1};
        pose.translation = Math3.times(Math3.mv(pose.rotation, centre), -1);
        double[] distortion = (i + j) % 4 == 1 ? new double[] {-.06, .01, .0005, -.0004, 0} : new double[5];
        Orthomosaic.View view =
            new Orthomosaic.View("camera" + views.size(), pose, new Calibration(400, 400, 199.5, 199.5, distortion, SIZE, SIZE), 1);
        int[] image = new int[SIZE * SIZE];
        for (int v = 0; v < SIZE; v++) for (int u = 0; u < SIZE; u++) image[v * SIZE + u] = cast(view.centre, view.ray(u, v));
        views.add(view);
        images.add(image);
      }
    check(Math.abs(views.get(4).centre[0] - 25) < 1e-9 && Math.abs(views.get(4).centre[2] - 50) < 1e-9, "camera centre from the pose");
    double[] uv = new double[2];
    check(views.get(0).project(10, 5, 0, uv) && Math.abs(uv[0] - 199.5) < 1e-9 && Math.abs(uv[1] - 199.5) < 1e-9, "nadir point projects to the principal point");
    check(views.get(0).project(20, 5, 0, uv) && Math.abs(uv[0] - 279.5) < 1e-9, "10 m east at 50 m is 80 pixels right");
    check(views.get(0).project(10, 15, 0, uv) && Math.abs(uv[1] - 119.5) < 1e-9, "north is up in the image");
    // Distorted view: ray() inverts project().
    Orthomosaic.View bent = views.get(1);
    double[] direction = bent.ray(40, 350);
    double scale = -50 / direction[2];
    check(bent.project(bent.centre[0] + scale * direction[0], bent.centre[1] + scale * direction[1], 0, uv)
        && Math.abs(uv[0] - 40) < 1e-6 && Math.abs(uv[1] - 350) < 1e-6, "undistorted ray projects back to its pixel");
    check(!views.get(0).project(10, 5, 60, uv), "a point behind the camera does not project");
    Orthomosaic.Photos memory = (view, x, y, w, h) -> {
      int[] out = new int[w * h];
      for (int row = 0; row < h; row++) System.arraycopy(images.get(view), (y + row) * SIZE + x, out, row * w, w);
      return out;
    };

    // ---- Surface from the cloud: ground and roof every 0.2 m; no ground from x = 36 to 37.5.
    Orthomosaic.Cloud cloud = (label, consumer) -> {
      for (double y = -1.9; y < 42; y += .2)
        for (double x = 3.1; x < 47; x += .2) {
          boolean inside = x >= BX0 && x <= BX1 && y >= BY0 && y <= BY1;
          if (inside) consumer.point(x, y, ROOF);
          else if (!(x > BX1 && x < 37.5 && y >= BY0 && y <= BY1)) consumer.point(x, y, 0);
        }
      consumer.point(Double.NaN, 1, 1);
    };
    Orthomosaic.Surface surface = new Orthomosaic.Surface(180, 180, 3, 42, .25);
    surface.add(cloud, "test surface");
    long before = 0;
    for (float v : surface.z) if (!Float.isNaN(v)) before++;
    surface.finish(8);
    check(surface.filled > 0 && surface.known == before + surface.filled, "gap cells filled (" + surface.filled + ")");
    check(surface.top == (float) ROOF && surface.bottom == 0, "height range of the surface");
    check(surface.height(33, 27) == ROOF && surface.height(10, 10) == 0, "roof and ground heights");
    check(surface.height(36.8, 27) == 0 && surface.height(37.3, 25) == 0, "the gap beside the box continues the ground, not the roof");
    check(surface.height(29.7, 27) == 0 && surface.height(30.3, 27) == ROOF, "no ramp at the roof step");
    check(Double.isNaN(new Orthomosaic.Surface(4, 4, 0, 4, 1).height(2, 2)), "unknown surface has no height");
    Orthomosaic.Surface slope = new Orthomosaic.Surface(4, 4, 0, 4, 1);
    for (int p = 0; p < 16; p++) slope.z[p] = p % 4;
    slope.finish(0);
    check(Math.abs(slope.height(2, 2) - 1.5) < 1e-9 && Math.abs(slope.height(1.25, 3.1) - .75) < 1e-9, "bilinear on a continuous slope");
    // Heights on both sides of zero: the gap still continues the lower side.
    Orthomosaic.Surface signs = new Orthomosaic.Surface(30, 12, 0, 12, 1);
    for (int p = 0; p < 360; p++) signs.z[p] = p % 30 < 12 ? -.5f : p % 30 >= 18 ? 3 : Float.NaN;
    signs.finish(8);
    check(signs.height(12.5, 6) == -.5 && signs.height(17.5, 6) == -.5 && signs.filled == 48, "a gap between heights below and above zero continues the lower side");
    check(surface.visible(new double[] {25, 27, 50}, 28, 27, 0, false), "ground in front of the box is visible");
    check(!surface.visible(new double[] {25, 27, 50}, 38, 27, 0, false), "ground behind the box is hidden");
    check(surface.visible(new double[] {40, 27, 50}, 38, 27, 0, false), "... and visible from the other side");
    check(surface.visible(new double[] {25, 27, 50}, 33, 27, ROOF, false), "the roof is visible");
    check(surface.visible(new double[] {33, 27, 50}, 33.05, 27, ROOF, false), "straight down is visible");

    // ---- Mosaic: 2,500 x 2,300 pixels of 0.016 m from (5, 38.8): four blocks.
    double x0 = 5, yTop = 38.8, cell = .016;
    int width = 2500, height = 2300;
    Path root = Files.createTempDirectory(Files.createDirectories(Path.of("target")), "orthomosaic ");
    try {
      Path file = root.resolve("ortho.tif"), bigFile = root.resolve("ortho-big.tif");
      Map<Long, byte[]> tiles = new HashMap<>();
      Orthomosaic.Result result;
      try (TiledGeoTiff tiff = new TiledGeoTiff(file, width, height, Orthomosaic.TILE, x0, yTop, cell, 3067, false);
          TiledGeoTiff big = new TiledGeoTiff(bigFile, width, height, Orthomosaic.TILE, x0, yTop, cell, 3067, true)) {
        result = Orthomosaic.render(surface, views, memory, x0, yTop, cell, width, height, 8, true, (tx, ty, bytes) -> {
          tiles.put((long) ty << 32 | tx, bytes);
          tiff.write(tx, ty, bytes);
          big.write(tx, ty, bytes);
        });
      }
      check(tiles.size() == 5 * 5, "25 tiles of 512 pixels");
      Map<Long, byte[]> single = new HashMap<>();
      Orthomosaic.Result one =
          Orthomosaic.render(surface, views, memory, x0, yTop, cell, width, height, 1, true, (tx, ty, bytes) -> single.put((long) ty << 32 | tx, bytes));
      boolean same = single.size() == tiles.size() && one.painted == result.painted;
      for (var e : tiles.entrySet()) same &= Arrays.equals(e.getValue(), single.get(e.getKey()));
      check(same, "tiles do not depend on the thread count");
      BufferedImage read = ImageIO.read(file.toFile());
      check(read != null && read.getWidth() == width && read.getHeight() == height && read.getColorModel().hasAlpha(),
          "GeoTIFF read back by the JDK decoder with alpha");
      long ground = 0, roofPixels = 0, wrong = 0, clear = 0, wall = 0, outline = 0;
      int worst = 0;
      double[] box = {1e9, 1e9, -1e9, -1e9};
      for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++) {
          double x = x0 + (col + .5) * cell, y = yTop - (row + .5) * cell;
          int argb = read.getRGB(col, row), colour = argb & 0xffffff;
          double outside = Math.max(Math.max(BX0 - x, x - BX1), Math.max(BY0 - y, y - BY1));
          if (Math.abs(outside) < .5) {
            outline++;
            continue;
          }
          if (argb >>> 24 == 0) {
            clear++;
            continue;
          }
          if (close(colour, WALL, 12)) wall++;
          int expected = outside > 0 ? ground(x, y) : roof(x, y);
          if (outside > 0) ground++;
          else roofPixels++;
          if (!close(colour, expected, 8)) {
            wrong++;
            box[0] = Math.min(box[0], x);
            box[1] = Math.min(box[1], y);
            box[2] = Math.max(box[2], x);
            box[3] = Math.max(box[3], y);
          }
          for (int shift = 0; shift <= 16; shift += 8)
            worst = Math.max(worst, Math.abs(((colour >> shift) & 255) - ((expected >> shift) & 255)));
        }
      check(clear == 0, "nine cameras: every pixel away from the outline is painted (" + clear + " transparent)");
      check(wrong == 0, "every ground and roof pixel shows its own position's colour (" + wrong + " wrong within " + Arrays.toString(box) + ", worst channel difference " + worst + ")");
      check(wall == 0, "no wall colour away from the outline");
      check(roofPixels > 90_000 && ground > 5_000_000, "roof and ground pixels counted: " + roofPixels + ", " + ground);
      check(result.painted + result.unknown + result.unseen == (long) width * height && result.unknown == 0, "result counts add up");
      int contributing = 0;
      for (long n : result.pixels) if (n > 0) contributing++;
      check(contributing == 9, "all nine cameras contribute");

      // ---- Two western cameras only: the ground behind the box is hidden from both.
      List<Orthomosaic.View> west = List.of(views.get(3), views.get(4));
      Orthomosaic.Photos westPhotos = (view, x, y, w, h) -> memory.window(view + 3, x, y, w, h);
      int[] shadow = strip(surface, west, westPhotos, true), leaning = strip(surface, west, westPhotos, false);
      check(shadow[0] > 20_000 && shadow[1] == 0, "true orthomosaic: hidden ground is transparent, nothing wrong (" + shadow[0] + " transparent, " + shadow[1] + " wrong)");
      check(leaning[1] > 20_000 && leaning[0] == 0, "without the visibility test the box covers that ground (" + leaning[1] + " wrong pixels)");

      // ---- GeoTIFF tags and the BigTIFF variant.
      Map<Integer, double[]> tags = tags(file, false), bigTags = tags(bigFile, true);
      check(tags.get(256)[0] == width && tags.get(257)[0] == height && tags.get(322)[0] == 512 && tags.get(259)[0] == 8, "size, tile and Deflate tags");
      check(tags.get(277)[0] == 4 && tags.get(338)[0] == 2 && tags.get(262)[0] == 2, "RGB with unassociated alpha");
      check(tags.get(33550)[0] == cell && tags.get(33550)[1] == cell, "pixel scale tag");
      check(tags.get(33922)[3] == x0 && tags.get(33922)[4] == yTop, "tie point at the top-left outer corner");
      check(tags.get(34735)[15] == 3067 && tags.get(34735)[7] == 1, "projected CRS key");
      check(tags.get(324).length == 25 && bigTags.get(324).length == 25, "tile offsets");
      boolean equal = true;
      try (RandomAccessFile a = new RandomAccessFile(file.toFile(), "r"); RandomAccessFile b = new RandomAccessFile(bigFile.toFile(), "r")) {
        for (int t = 0; t < 25; t++) {
          byte[] expected = tiles.get((long) (t / 5) << 32 | (t % 5));
          byte[] fromClassic = new byte[(int) tags.get(325)[t]], fromBig = new byte[(int) bigTags.get(325)[t]];
          a.seek((long) tags.get(324)[t]);
          a.readFully(fromClassic);
          b.seek((long) bigTags.get(324)[t]);
          b.readFully(fromBig);
          equal &= Arrays.equals(expected, fromClassic) && Arrays.equals(expected, fromBig);
        }
      }
      check(equal, "classic and BigTIFF files hold the rendered tiles");
      for (int id : new int[] {256, 257, 258, 259, 262, 277, 284, 322, 323, 338, 339, 33550, 33922, 34735})
        check(Arrays.equals(tags.get(id), bigTags.get(id)), "BigTIFF tag " + id + " equals the classic one");
      try (var input = ImageIO.createImageInputStream(bigFile.toFile())) {
        BufferedImage part = new BigTiffWindows(input).read(new Rectangle(1500, 400, 300, 200));
        boolean matches = part.getWidth() == 300 && part.getHeight() == 200;
        for (int y = 0; y < 200 && matches; y += 7)
          for (int x = 0; x < 300 && matches; x += 5) matches = part.getRGB(x, y) == read.getRGB(1500 + x, 400 + y);
        check(matches, "the BigTIFF decodes to the same pixels");
      }
      boolean refused = false;
      try (TiledGeoTiff open = new TiledGeoTiff(root.resolve("incomplete.tif"), 600, 600, 512, 0, 0, 1, 0, false)) {
        open.write(0, 0, TiledGeoTiff.compress(new byte[512 * 512 * 4]));
      } catch (java.io.IOException incomplete) {
        refused = true;
      }
      check(refused, "closing with missing tiles is refused");
      check(!TiledGeoTiff.needsBig(20_000, 20_000) && TiledGeoTiff.needsBig(40_000, 30_000), "BigTIFF above about 3.5 GB of samples");

      // ---- Photographs from files: whole images with a small cache, and by window.
      List<Path> files = new ArrayList<>();
      int[] sizes = new int[9];
      Arrays.fill(sizes, SIZE);
      for (int i = 0; i < 9; i++) {
        BufferedImage image = new BufferedImage(SIZE, SIZE, BufferedImage.TYPE_INT_RGB);
        image.setRGB(0, 0, SIZE, SIZE, images.get(i), 0, SIZE);
        Path photo = root.resolve("photo" + i + ".png");
        ImageIO.write(image, "png", photo.toFile());
        files.add(photo);
      }
      try (Orthomosaic.RunPhotos whole = new Orthomosaic.RunPhotos(files, sizes, sizes, 16L * SIZE * SIZE);
          Orthomosaic.RunPhotos windowed = new Orthomosaic.RunPhotos(files, sizes, sizes, 1000)) {
        boolean matches = true;
        for (int round = 0; round < 2; round++)
          for (int i = 0; i < 9; i++) {
            int[] expected = memory.window(i, 37, 101, 200, 150);
            int[] a = whole.window(i, 37, 101, 200, 150), b = windowed.window(i, 37, 101, 200, 150);
            for (int p = 0; p < expected.length; p++) matches &= (a[p] & 0xffffff) == expected[p] && (b[p] & 0xffffff) == expected[p];
          }
        check(matches, "photograph windows from files equal the images");
        check(whole.decoded == 18 && whole.windows == 0, "a cache for four images decodes again after eviction (" + whole.decoded + ")");
        check(windowed.decoded == 0 && windowed.windows == 18, "images above the whole-image limit are read by window");
      }
    } finally {
      try (var walk = Files.walk(root)) {
        for (Path p : walk.sorted(Comparator.reverseOrder()).toList()) Files.deleteIfExists(p);
      }
    }
    check(Orthomosaic.twoDigits(.03933) == .04 && Math.abs(Orthomosaic.twoDigits(.1234) - .13) < 1e-12 && Orthomosaic.twoDigits(5) == 5, "pixel size rounded up to two digits");
    Options parsed = Options.parse(new String[] {"-i", ".", "--orthomosaic", "--orthomosaic-resolution", "0.05", "--orthomosaic-fill", "0"});
    check(parsed.orthomosaic && parsed.orthomosaicResolution == .05 && parsed.orthomosaicFill == 0 && !new Options().orthomosaic, "orthomosaic options; off by default");
    check(!parsed.orthomosaicStrict && Options.parse(new String[] {"-i", ".", "--orthomosaic", "--orthomosaic-strict"}).orthomosaicStrict, "--orthomosaic-strict; off by default");

    canopy();
    checkPoints();
    return checks;
  }

  /**
   * Canopy. Conifers (cones of slope 4, crowns of 1.2 m radius, apexes 12..18 m) on a 3 m lattice
   * with only their crowns in the cloud, a strip of ground in the west, and in the north-east
   * corner a box of 4 x 4 x 10 m on ground that has no points just east of the box. Two cameras
   * in the west, 40 m up: the east flanks of the crowns and the gaps between them fail the
   * line-of-sight test, and so does the ground behind the box.
   *
   * <ul>
   *   <li>crowns are SOFT, the roof (edges included) and the ground are not;
   *   <li>a gap between crowns takes the height of the nearest crown, a gap next to ground the
   *       ground;
   *   <li>strict: the stand is full of holes; default: the stand is painted (all but 0.2%), the
   *       counts add up, and the ground behind the box is as transparent as before;
   *   <li>the same tiles on 1 and 8 threads.
   * </ul>
   */
  private static void canopy() throws Exception {
    List<double[]> trees = new ArrayList<>();
    for (int j = 0; j < 8; j++)
      for (int i = 0; i < 8; i++) {
        if (i >= 5 && j >= 5) continue;
        int hash = (i * 73 + j * 151) % 17;
        trees.add(new double[] {8 + 3 * i + .05 * (hash % 5), 8 + 3 * j + .05 * (hash % 7), 12 + 6 * hash / 16.});
      }
    Orthomosaic.Cloud cloud = (label, consumer) -> {
      for (int b = 0; b < 660; b++)
        for (int a = 0; a < 660; a++) {
          double x = .025 + .05 * a, y = .025 + .05 * b;
          if (x < 5) consumer.point(x, y, 0);
          else if (x >= 26 && x <= 30 && y >= 26 && y <= 30) consumer.point(x, y, 10);
          else if (x >= 23 && y >= 23) {
            if (!(x > 30 && x < 31.5 && y >= 26 && y <= 30)) consumer.point(x, y, 0);
          } else
            for (double[] t : trees) {
              double d = Math.hypot(x - t[0], y - t[1]);
              if (d <= 1.2) consumer.point(x, y, t[2] - 4 * d);
            }
        }
    };
    Orthomosaic.Surface s = new Orthomosaic.Surface(330, 330, 0, 33, .1);
    s.add(cloud, "test canopy");
    boolean[] original = new boolean[s.z.length];
    for (int p = 0; p < original.length; p++) original[p] = !Float.isNaN(s.z[p]);
    s.finish(20, 4);
    long crowns = 0, softCrowns = 0, softSolid = 0, solid = 0;
    for (int p = 0; p < original.length; p++) {
      if (!original[p]) continue;
      double x = (p % 330 + .5) * .1, y = 33 - (p / 330 + .5) * .1;
      boolean soft = (s.kind[p] & Orthomosaic.Surface.SOFT) != 0;
      if (s.z[p] > 1 && s.z[p] != 10) {
        crowns++;
        if (soft) softCrowns++;
      } else if (x < 4.5 || (x >= 24 && y >= 24)) {
        solid++;
        if (soft) softSolid++;
      }
    }
    check(crowns > 20_000 && softCrowns >= .95 * crowns, "crowns are rough on their upper side: " + softCrowns + " of " + crowns + " cells");
    check(solid > 20_000 && softSolid == 0, "the roof with its edges and the ground are solid (" + softSolid + " of " + solid + " cells rough)");
    check(s.soft >= softCrowns + s.filled && s.filled > 10_000, "filled cells count as canopy, not as solid");
    // Between the crowns of the trees at (14.05, 14.3) and (17.05, 14.2), far from any ground.
    double gap = s.height(15.55, 14.25);
    check(gap > 7 && gap < 14, "a gap between crowns takes the height of a crown edge, not of the lowest rim (" + gap + ")");
    check(s.height(5.5, 15) == 0 && s.height(6.2, 12.5) == 0, "a gap next to ground continues the ground");
    check(s.height(30.8, 28) == 0, "the gap beside the box continues the ground");
    double[] west = {-30, 14.25, 60};
    check(!s.visible(west, 15.55, 14.25, gap, false) && !s.visible(west, 15.55, 14.25, gap, true), "the gap is behind a crown for a camera in the west");
    double through = s.obstruction(west, 15.55, 14.25, gap, Double.POSITIVE_INFINITY);
    check(through > .5 && through < 10, "... with only canopy in the way (" + through + " m of it)");
    check(s.obstruction(west, 15.55, 14.25, gap, .25) == Double.POSITIVE_INFINITY, "an obstruction above the limit is not measured further");
    check(s.obstruction(new double[] {15.55, 14.25, 60}, 15.6, 14.25, gap, 1) == 0 && s.obstruction(west, 3, 15, 0, 1) == 0, "nothing in the way: no obstruction");
    check(s.obstruction(new double[] {-30, 28, 60}, 30.8, 28, 0, Double.POSITIVE_INFINITY) == Double.POSITIVE_INFINITY, "the roof in the way is solid");
    check(s.obstruction(west, 15.55, 14.25, 70, 1) == Double.POSITIVE_INFINITY, "a camera below the point sees nothing");

    List<Orthomosaic.View> views = new ArrayList<>();
    for (double y : new double[] {10, 23}) {
      Camera pose = new Camera(250, 250, 199.5, 199.5);
      pose.rotation = new double[] {1, 0, 0, 0, -1, 0, 0, 0, -1};
      pose.translation = Math3.times(Math3.mv(pose.rotation, new double[] {2, y, 40}), -1);
      views.add(new Orthomosaic.View("west" + views.size(), pose, new Calibration(250, 250, 199.5, 199.5, new double[5], SIZE, SIZE), 1));
    }
    int[] colours = {0x20c040, 0x2040c0};
    Orthomosaic.Photos photos = (view, x, y, w, h) -> {
      int[] out = new int[w * h];
      Arrays.fill(out, colours[view]);
      return out;
    };
    // 500 x 460 pixels of 0.05 m from (7, 30): one tile.
    byte[][] tile = new byte[3][];
    Orthomosaic.Result strict = Orthomosaic.render(s, views, photos, 7, 30, .05, 500, 460, 8, true, false, (tx, ty, bytes) -> tile[0] = bytes);
    Orthomosaic.Result open = Orthomosaic.render(s, views, photos, 7, 30, .05, 500, 460, 8, true, true, (tx, ty, bytes) -> tile[1] = bytes);
    Orthomosaic.Result single = Orthomosaic.render(s, views, photos, 7, 30, .05, 500, 460, 1, true, true, (tx, ty, bytes) -> tile[2] = bytes);
    check(Arrays.equals(tile[1], tile[2]) && single.relaxed == open.relaxed, "canopy tiles do not depend on the thread count");
    check(strict.relaxed == 0 && open.relaxed > 0 && open.painted == strict.painted + open.relaxed && open.unseen == strict.unseen - open.relaxed && open.unknown == strict.unknown,
        "pixels painted through canopy are counted (" + open.relaxed + " of " + strict.unseen + " unseen)");
    byte[] before = inflate(tile[0]), after = inflate(tile[1]);
    long stand = 0, holes = 0, left = 0, behind = 0, hidden = 0, changed = 0;
    for (int row = 0; row < 460; row++)
      for (int col = 0; col < 500; col++) {
        double x = 7 + (col + .5) * .05, y = 30 - (row + .5) * .05;
        int at = 4 * (row * 512 + col);
        if (before[at + 3] != 0 && (after[at] != before[at] || after[at + 1] != before[at + 1] || after[at + 2] != before[at + 2] || after[at + 3] != before[at + 3])) changed++;
        if (x >= 8 && x <= 21 && y >= 9 && y <= 21) {
          stand++;
          if (before[at + 3] == 0) holes++;
          if (after[at + 3] == 0) left++;
        }
        if (x >= 30.3 && x <= 31.9 && y >= 27.5 && y <= 29.5) {
          behind++;
          if (after[at + 3] == 0 && before[at + 3] == 0) hidden++;
        }
      }
    check(holes > .05 * stand, "strict: the stand has holes (" + holes + " of " + stand + " pixels)");
    check(500 * left < stand && 50 * left < holes, "default: the stand is painted but for pixels behind the few smooth crown cells (" + left + " of " + stand + " transparent, strict " + holes + ")");
    check(behind > 1000 && hidden == behind, "the ground behind the box stays transparent next to canopy (" + hidden + " of " + behind + ")");
    check(changed == 0, "pixels that pass the line-of-sight test are the same (" + changed + " changed)");
  }

  private static byte[] inflate(byte[] bytes) throws Exception {
    java.util.zip.Inflater inflater = new java.util.zip.Inflater();
    inflater.setInput(bytes);
    byte[] samples = new byte[512 * 512 * 4];
    try {
      int got = 0;
      while (got < samples.length && !inflater.finished()) got += inflater.inflate(samples, got, samples.length - got);
    } finally {
      inflater.end();
    }
    return samples;
  }

  /** Transparent and wrong pixels in the strip of ground just east of the box (x 36.6..38). */
  private static int[] strip(Orthomosaic.Surface surface, List<Orthomosaic.View> views, Orthomosaic.Photos photos, boolean occlusion)
      throws Exception {
    double x0 = 36.6, yTop = 29, cell = .01; // 36.6..37.6 east of the box, y 25..29
    int width = 100, height = 400;
    int[] counts = new int[2];
    for (Orthomosaic.View v : views) v.reach = null;
    Orthomosaic.render(surface, views, photos, x0, yTop, cell, width, height, 4, occlusion, (tx, ty, bytes) -> {
      java.util.zip.Inflater inflater = new java.util.zip.Inflater();
      inflater.setInput(bytes);
      byte[] samples = new byte[512 * 512 * 4];
      try {
        int got = 0;
        while (got < samples.length && !inflater.finished()) got += inflater.inflate(samples, got, samples.length - got);
      } catch (java.util.zip.DataFormatException e) {
        throw new java.io.IOException(e);
      } finally {
        inflater.end();
      }
      for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++) {
          int at = 4 * (row * 512 + col);
          int colour = (samples[at] & 255) << 16 | (samples[at + 1] & 255) << 8 | (samples[at + 2] & 255);
          if (samples[at + 3] == 0) counts[0]++;
          else if (!close(colour, ground(x0 + (col + .5) * cell, yTop - (row + .5) * cell), 8)) counts[1]++;
        }
    });
    return counts;
  }

  /** Numeric values of the tags of a little-endian TIFF (classic or BigTIFF) directory. */
  private static Map<Integer, double[]> tags(Path file, boolean big) throws Exception {
    Map<Integer, double[]> out = new HashMap<>();
    try (RandomAccessFile in = new RandomAccessFile(file.toFile(), "r")) {
      byte[] all = new byte[(int) in.length()];
      in.readFully(all);
      ByteBuffer b = ByteBuffer.wrap(all).order(ByteOrder.LITTLE_ENDIAN);
      if (b.get(0) != 'I' || b.getShort(2) != (big ? 43 : 42)) throw new AssertionError("TIFF header of " + file);
      long directory = big ? b.getLong(8) : b.getInt(4) & 0xffffffffL;
      long entries = big ? b.getLong((int) directory) : b.getShort((int) directory) & 0xffff;
      int at = (int) directory + (big ? 8 : 2), entry = big ? 20 : 12, inline = big ? 8 : 4;
      for (int e = 0; e < entries; e++, at += entry) {
        int id = b.getShort(at) & 0xffff, type = b.getShort(at + 2);
        int count = (int) (big ? b.getLong(at + 4) : b.getInt(at + 4));
        int size = type == 3 ? 2 : type == 4 ? 4 : type == 12 || type == 16 ? 8 : 1;
        int data = at + (big ? 12 : 8);
        if ((long) size * count > inline) data = (int) (big ? b.getLong(data) : b.getInt(data));
        double[] values = new double[count];
        for (int i = 0; i < count; i++)
          values[i] =
              type == 3 ? b.getShort(data + 2 * i) & 0xffff
                  : type == 4 ? b.getInt(data + 4 * i) & 0xffffffffL
                      : type == 12 ? b.getDouble(data + 8 * i)
                          : type == 16 ? b.getLong(data + 8 * i) : b.get(data + i);
        out.put(id, values);
      }
    }
    return out;
  }

  private static void checkPoints() {
    // A 3 x 3 grid of control points plus one typed check in the file.
    List<GroundControl.Point3> points = new ArrayList<>();
    double[] sigma = {.02, .02, .03};
    for (int j = 0; j < 3; j++)
      for (int i = 0; i < 3; i++) points.add(new GroundControl.Point3("p" + (3 * j + i), new double[] {100 * i, 100 * j, 5}, sigma, false));
    points.add(new GroundControl.Point3("given", new double[] {50, 50, 5}, sigma, true));
    GroundControl.Selection none = GroundControl.holdOut(points, "none", List.of());
    check(none.points() == points && none.note() == null, "no option: the file's types, no note");
    GroundControl.Selection auto = GroundControl.holdOut(points, "auto", List.of());
    List<String> held = new ArrayList<>();
    for (GroundControl.Point3 p : auto.points()) if (p.check) held.add(p.id);
    // Nine control points: two held back; the corners and the centre are spread widest.
    check(held.size() == 3 && held.contains("given"), "auto holds two of nine back, next to the given check point: " + held);
    for (String corner : List.of("p0", "p2", "p6", "p8", "p4")) check(!held.contains(corner), "auto keeps " + corner + " as control");
    check(auto.note().contains("--gcp-check auto") && auto.note().contains(held.get(0)), "the note lists the points held back");
    check(auto.points().get(0).xyz == points.get(0).xyz && auto.points().size() == points.size(), "coordinates and order unchanged");
    GroundControl.Selection again = GroundControl.holdOut(points, "auto", List.of());
    boolean stable = true;
    for (int i = 0; i < points.size(); i++) stable &= again.points().get(i).check == auto.points().get(i).check;
    check(stable, "the choice is repeatable");
    GroundControl.Selection four = GroundControl.holdOut(points, "4", List.of());
    int control = 0;
    for (GroundControl.Point3 p : four.points()) if (!p.check) control++;
    check(control == 5, "--gcp-check 4 leaves five control points");
    for (String corner : List.of("p0", "p2", "p6", "p8")) check(!four.points().get(Integer.parseInt(corner.substring(1))).check, "count: corner " + corner + " stays control");
    GroundControl.Selection named = GroundControl.holdOut(points, "none", List.of("p4", "given"));
    check(named.points().get(4).check && named.note().contains("p4") && !named.note().contains("given"), "--gcp-check-ids turns p4 into a check point");
    boolean refused = false;
    try {
      GroundControl.holdOut(points, "7", List.of());
    } catch (IllegalArgumentException e) {
      refused = e.getMessage().contains("at least 3");
    }
    check(refused, "a count that leaves fewer than three control points is refused");
    refused = false;
    try {
      GroundControl.holdOut(points, "none", List.of("missing"));
    } catch (IllegalArgumentException e) {
      refused = true;
    }
    check(refused, "an unknown id is refused");
    GroundControl.Selection few = GroundControl.holdOut(points.subList(0, 4), "auto", List.of());
    check(few.points() == points.subList(0, 4) || few.points().stream().noneMatch(p -> p.check), "auto holds nothing back with four control points");
    check(few.note() != null && few.note().contains("fewer than five"), "... and says so");
    check(GroundControl.ids(" a, b ,,c").equals(List.of("a", "b", "c")), "id list parsing");
    Options o = Options.parse(new String[] {"-i", ".", "--gcp-check", "auto", "--gcp-check-ids", "a,b"});
    check(o.gcpCheck.equals("auto") && o.gcpCheckIds.equals("a,b") && new Options().gcpCheck.equals("none"), "--gcp-check options; none by default");
    refused = false;
    try {
      Options.parse(new String[] {"-i", ".", "--gcp-check", "some"});
    } catch (IllegalArgumentException e) {
      refused = true;
    }
    check(refused, "an unknown --gcp-check value is rejected");
  }
}
