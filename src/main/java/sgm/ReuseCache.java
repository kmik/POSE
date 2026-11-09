package sgm;

import java.io.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;
import org.opencv.core.*;

/**
 * Project-local immutable entries. Complete payload and artifacts are SHA-256 verified on reuse.
 */
final class ReuseCache {
  interface Writer {
    void write(DataOutputStream out) throws IOException;
  }

  interface Reader {
    void read(DataInputStream in) throws IOException;
  }

  static String hash(Path file) throws IOException {
    try (InputStream in = Files.newInputStream(file)) {
      MessageDigest d = digest();
      byte[] b = new byte[65536];
      int n;
      while ((n = in.read(b)) >= 0) d.update(b, 0, n);
      return HexFormat.of().formatHex(d.digest());
    }
  }

  private static MessageDigest digest() {
    try {
      return MessageDigest.getInstance("SHA-256");
    } catch (NoSuchAlgorithmException e) {
      throw new AssertionError(e);
    }
  }

  private static final String ENGINE = engine();

  static String engineIdentifier() {
    return ENGINE;
  }

  private static String engine() {
    MessageDigest d = digest();
    for (Class<?> c :
        new Class<?>[] {
          Options.class,
          Pipeline.class,
          PipelineState.class,
          Camera.class,
          DisparityRange.class,
          Geometry.class,
          FeatureTracks.class,
          ImageFrame.class,
          WindowedImage.class,
          BigTiffWindows.class,
          ImageFusion.class,
          TsgmSearch.class,
          BandedSgm.class,
          GpuBandedStereo.class,
          DisparityBands.class,
          PairMatching.class,
          RasterCache.class,
          DescriptorCache.class,
          GpuAggregate.class,
          GpuCandidateStereo.class,
          AutomaticCalibration.class,
          SelfCalibration.class,
          BundleAdjuster.class,
          SharedCalibration.class,
          CalibrationObservability.class,
          CalibrationDiagnostics.class,
          PointConfidence.class,
          PointRecords.class,
          SmallMatrices.class,
          TwoViewConfidence.class,
          FeatureAlgorithms.class,
          PairSelection.class,
          DensePairSelection.class,
          CameraEmission.class,
          CameraEvidence.class,
          MemorySafety.class,
          WindowedRasters.class,
          WindowedStereo.class,
          DepthOutlierFilter.class,
          SurfacePatchGrid.class,
          AerialTileStereo.class,
          AerialRectifier.class,
          AerialPairMap.class,
          AerialTiledReconstruction.class,
          StereoRows.class,
          FusionScope.class,
          IncrementalFusion.class,
          SparseCleanup.class,
          DenseReconstruction.class,
          CensusSgm.class,
          GpuSgm.class,
          GpuStereo.class,
          StereoCost.class,
          ZnccCosts.class,
          PhotometricRefinement.class,
          MultiViewDisparityRefinement.class,
          OverlapSelection.class,
          OrientationIO.class,
          OrientationText.class,
          OrientationJson.class,
          OrientationXml.class,
          FusionPipeline.class,
          GpuFusion.class,
          GpuSurface.class,
          DenseMatcher.class,
          PatchMatch.class,
          PatchMatchReconstruction.class,
          DepthFusion.class,
          ReprojectionDepth.class,
          SurfaceGeometry.class,
          SurfaceAttributes.class,
          SurfaceConsolidator.class,
          FusionOutput.class,
          PagedVolume.class,
          DepthRaster.class,
          PoseIO.class,
          Calibration.class,
          Math3.class,
          Exif.class,
          ReuseCache.class
        }) {
      fingerprint(c, d);
    }
    return HexFormat.of().formatHex(d.digest());
  }

  private static void fingerprint(Class<?> c, MessageDigest d) {
    try (InputStream in = c.getResourceAsStream("/" + c.getName().replace('.', '/') + ".class")) {
      if (in != null) d.update(in.readAllBytes());
      for (Class<?> nested : c.getDeclaredClasses()) fingerprint(nested, d);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  static String key(String text) {
    return HexFormat.of()
        .formatHex(
            digest()
                .digest(
                    ("sgm-todo-v4|" + ENGINE + "|" + Core.VERSION + "|" + text)
                        .getBytes(java.nio.charset.StandardCharsets.UTF_8)));
  }

  static String bytes(byte[]... arrays) {
    MessageDigest d = digest();
    for (byte[] b : arrays) if (b != null) d.update(b);
    return HexFormat.of().formatHex(d.digest());
  }

  static String settings(Options o, String... names) {
    StringBuilder b = new StringBuilder();
    try {
      for (String n : names)
        b.append(n).append('=').append(Options.class.getField(n).get(o)).append(';');
    } catch (ReflectiveOperationException e) {
      throw new IllegalArgumentException(e);
    }
    return b.toString();
  }

  static String frame(ImageFrame f) throws IOException {
    fingerprint(f);
    return key(
        f.id
            + ":"
            + f.path.getFileName()
            + ":"
            + f.width
            + ":"
            + f.height
            + ":"
            + f.camera.fx
            + ":"
            + f.camera.fy
            + ":"
            + f.camera.cx
            + ":"
            + f.camera.cy
            + ":"
            + f.pixelFingerprint);
  }

  private static void fingerprint(ImageFrame f) throws IOException {
    if (f.pixelFingerprint == null) {
      f.ensureRasters();
      MessageDigest d = digest();
      byte[] row = new byte[f.width];
      for (int y = 0; y < f.height; y++) {
        f.gray.get(y, 0, row);
        d.update(row);
        f.validMask.get(y, 0, row);
        d.update(row);
      }
      f.pixelFingerprint = HexFormat.of().formatHex(d.digest());
    }
  }

  static boolean allowed(Options o, String stage) {
    List<String> order =
        List.of("calibration", "features", "matches", "geometry", "stereo", "fusion");
    return o.resume
        && (o.rerunFrom.equals("none") || order.indexOf(stage) < order.indexOf(o.rerunFrom));
  }

  static boolean read(Options o, String stage, String key, Path artifacts, Reader reader)
      throws IOException {
    if (o.cacheDirectory == null || !allowed(o, stage)) return false;
    Path base = o.cacheDirectory.resolve(stage).resolve(key);
    if (!Files.isRegularFile(base.resolve("active"))) return false;
    Path entry = base.resolve(Files.readString(base.resolve("active")).trim()).normalize();
    if (!entry.startsWith(base) || !Files.isRegularFile(entry.resolve("complete.properties")))
      return false;
    try {
      Properties p = new Properties();
      try (ReaderIgnore r = new ReaderIgnore(entry.resolve("complete.properties"))) {
        p.load(r.reader);
      }
      for (String name : p.stringPropertyNames()) {
        Path file = entry.resolve(name).normalize();
        if (!file.startsWith(entry)
            || !Files.isRegularFile(file)
            || !hash(file).equals(p.getProperty(name)))
          throw new IOException("Cache checksum mismatch: " + name);
      }
      if (!p.containsKey("payload.bin")) throw new IOException("Missing cache payload");
      try (DataInputStream in =
          new DataInputStream(
              new BufferedInputStream(Files.newInputStream(entry.resolve("payload.bin"))))) {
        if (in.readInt() != 0x53474d32) throw new IOException("Unknown cache version");
        reader.read(in);
        if (in.read() != -1) throw new IOException("Trailing cache bytes");
      }
      if (artifacts != null && Files.isDirectory(entry.resolve("artifacts")))
        copy(entry.resolve("artifacts"), artifacts);
      System.out.println("Reused " + stage + " cache " + key.substring(0, 12));
      return true;
    } catch (IOException | RuntimeException e) {
      System.err.println("Ignoring incomplete/incompatible " + stage + " cache: " + e.getMessage());
      return false;
    }
  }

  // Small wrapper keeps java.io.Reader distinct from the binary callback interface above.
  private static final class ReaderIgnore implements AutoCloseable {
    final java.io.Reader reader;

    ReaderIgnore(Path path) throws IOException {
      reader = Files.newBufferedReader(path);
    }

    public void close() throws IOException {
      reader.close();
    }
  }

  static void write(Options o, String stage, String key, Path artifacts, Writer writer)
      throws IOException {
    if (o.cacheDirectory == null) return;
    Path parent = o.cacheDirectory.resolve(stage).resolve(key);
    Files.createDirectories(parent);
    // A force-rerun replaces the completed entry only after publishing a new complete payload.
    Path temp = Files.createTempDirectory(parent, "pending-");
    try (DataOutputStream out =
        new DataOutputStream(
            new BufferedOutputStream(Files.newOutputStream(temp.resolve("payload.bin"))))) {
      out.writeInt(0x53474d32);
      writer.write(out);
    }
    if (artifacts != null && Files.isDirectory(artifacts))
      copy(artifacts, temp.resolve("artifacts"));
    Properties p = new Properties();
    try (var files = Files.walk(temp)) {
      for (Path file : files.filter(Files::isRegularFile).toList())
        p.setProperty(temp.relativize(file).toString(), hash(file));
    }
    try (java.io.Writer out = Files.newBufferedWriter(temp.resolve("complete.properties"))) {
      p.store(out, "SHA-256 of every cached payload and artifact");
    }
    Path pointer = parent.resolve("active.pending");
    Files.writeString(pointer, temp.getFileName().toString());
    try {
      Files.move(
          pointer,
          parent.resolve("active"),
          StandardCopyOption.ATOMIC_MOVE,
          StandardCopyOption.REPLACE_EXISTING);
    } catch (AtomicMoveNotSupportedException e) {
      Files.move(pointer, parent.resolve("active"), StandardCopyOption.REPLACE_EXISTING);
    }
    try (var entries = Files.list(parent)) {
      for (Path old : entries.filter(Files::isDirectory).toList())
        if (!old.equals(temp)
            && old.getFileName().toString().startsWith("pending-")
            && Files.exists(old.resolve("complete.properties"))) {
          try (var files = Files.walk(old)) {
            for (Path path : files.sorted(Comparator.reverseOrder()).toList()) Files.delete(path);
          }
        }
    }
  }

  private static void copy(Path source, Path target) throws IOException {
    try (var files = Files.walk(source)) {
      for (Path f : files.filter(Files::isRegularFile).toList()) {
        Path to = target.resolve(source.relativize(f));
        Files.createDirectories(to.getParent());
        Files.copy(f, to, StandardCopyOption.REPLACE_EXISTING);
      }
    }
  }

  static void vector(DataOutputStream out, double[] x) throws IOException {
    out.writeInt(x == null ? -1 : x.length);
    if (x != null) for (double v : x) out.writeDouble(v);
  }

  static double[] vector(DataInputStream in) throws IOException {
    int n = in.readInt();
    if (n == -1) return null;
    if (n < 0 || n > 10000000) throw new IOException("Invalid vector size");
    double[] x = new double[n];
    for (int i = 0; i < n; i++) x[i] = in.readDouble();
    return x;
  }

  static boolean features(ImageFrame f, Options o) throws IOException {
    if (o.cacheDirectory == null) {
      if (o.runFeatureDirectory != null) {
        Options shared = o.copy();
        shared.cacheDirectory = o.runFeatureDirectory;
        shared.runFeatureDirectory = null;
        shared.resume = true;
        shared.rerunFrom = "none";
        boolean hit = features(f, shared);
        if (hit) o.runFeatureHits.incrementAndGet();
        return hit;
      }
      f.extract(o.features);
      return false;
    }
    // Detection/description depends on the actual raster, not the focal guess, camera ID or
    // filename. Full pixel/mask hashing still prevents reuse after distortion or resize changes.
    // Matching/geometry continue to use frame(), which includes camera intrinsics and identity.
    fingerprint(f);
    String key = key(f.width + ":" + f.height + ":" + f.pixelFingerprint + featureSettings(o));
    boolean reused =
        read(
            o,
            "features",
            key,
            null,
            in -> {
              int n = in.readInt(), columns = in.readInt(), type = in.readInt();
              if (n < 0 || n > 1000000 || columns < 0 || columns > 1024)
                throw new IOException("Invalid descriptors");
              KeyPoint[] keys = new KeyPoint[n];
              for (int i = 0; i < n; i++)
                keys[i] =
                    new KeyPoint(
                        in.readFloat(),
                        in.readFloat(),
                        in.readFloat(),
                        in.readFloat(),
                        in.readFloat(),
                        in.readInt(),
                        in.readInt());
              Mat descriptors = new Mat(n, columns, type);
              try {
                if (descriptors.depth() == CvType.CV_8U) {
                  byte[] row = new byte[columns];
                  for (int i = 0; i < n; i++) {
                    in.readFully(row);
                    descriptors.put(i, 0, row);
                  }
                } else {
                  float[] row = new float[columns];
                  for (int i = 0; i < n; i++) {
                    for (int j = 0; j < columns; j++) row[j] = in.readFloat();
                    descriptors.put(i, 0, row);
                  }
                }
              } catch (IOException | RuntimeException e) {
                descriptors.release();
                throw e;
              }
              f.keypoints = keys;
              f.descriptors.release();
              f.descriptors = descriptors;
            });
    if (!reused) {
      f.extract(o.features);
      write(
          o,
          "features",
          key,
          null,
          out -> {
            out.writeInt(f.keypoints.length);
            out.writeInt(f.descriptors.cols());
            out.writeInt(f.descriptors.type());
            for (KeyPoint k : f.keypoints) {
              out.writeFloat((float) k.pt.x);
              out.writeFloat((float) k.pt.y);
              out.writeFloat(k.size);
              out.writeFloat(k.angle);
              out.writeFloat(k.response);
              out.writeInt(k.octave);
              out.writeInt(k.class_id);
            }
            if (f.descriptors.depth() == CvType.CV_8U) {
              byte[] row = new byte[f.descriptors.cols()];
              for (int i = 0; i < f.descriptors.rows(); i++) {
                f.descriptors.get(i, 0, row);
                out.write(row);
              }
            } else {
              float[] row = new float[f.descriptors.cols()];
              for (int i = 0; i < f.descriptors.rows(); i++) {
                f.descriptors.get(i, 0, row);
                for (float v : row) out.writeFloat(v);
              }
            }
          });
    }
    return reused;
  }

  static String matchesKey(List<ImageFrame> frames, Options o) throws IOException {
    StringBuilder key =
        new StringBuilder(
            settings(
                o,
                "matchingAlgorithm",
                "descriptorMatcher",
                "features",
                "ratio",
                "reprojection",
                "minMatches",
                "neighbors",
                "pairSelection",
                "pairCandidates",
                "minForwardOverlap",
                "minSideOverlap",
                "minPairOverlap",
                "overlapGroundZ",
                "flightHeading",
                "overlapUnknown"));
    key.append(featureSettings(o));
    for (ImageFrame f : frames) {
      key.append(frame(f));
      if (f.gps != null) key.append(f.gps.latitude).append(f.gps.longitude);
    }
    return key(key.toString());
  }

  private static String featureSettings(Options o) {
    return settings(
        o,
        "matchingAlgorithm",
        "features",
        "siftContrast",
        "siftEdgeThreshold",
        "orbFastThreshold",
        "briskThreshold",
        "akazeThreshold",
        "kazeThreshold");
  }

  static boolean matches(List<ImageFrame> frames, FeatureTracks features, Options o, boolean save)
      throws IOException {
    if (o.cacheDirectory == null) return false;
    String key = matchesKey(frames, o);
    Path artifacts = o.output.resolve("03_matches");
    if (!save)
      return read(
          o,
          "matches",
          key,
          artifacts,
          in -> {
            features.pairs.clear();
            features.tracks.clear();
            int np = in.readInt();
            if (np < 0 || np > frames.size() * (long) frames.size())
              throw new IOException("Invalid pair count");
            for (int i = 0; i < np; i++) {
              var p = new FeatureTracks.Pair(in.readInt(), in.readInt());
              p.rotation = vector(in);
              p.translation = vector(in);
              p.essential = vector(in);
              p.reason = in.readUTF();
              int n = in.readInt();
              for (int j = 0; j < n; j++)
                p.matches.add(new FeatureTracks.Match(in.readInt(), in.readInt()));
              features.pairs.add(p);
            }
            int nt = in.readInt();
            if (nt < 0 || nt > 10000000) throw new IOException("Invalid track count");
            for (int i = 0; i < nt; i++) {
              var t = new FeatureTracks.Track();
              t.id = in.readInt();
              int n = in.readInt();
              for (int j = 0; j < n; j++)
                t.observations.add(new FeatureTracks.Observation(in.readInt(), in.readInt()));
              features.tracks.add(t);
            }
          });
    write(
        o,
        "matches",
        key,
        artifacts,
        out -> {
          out.writeInt(features.pairs.size());
          for (var p : features.pairs) {
            out.writeInt(p.a);
            out.writeInt(p.b);
            vector(out, p.rotation);
            vector(out, p.translation);
            vector(out, p.essential);
            out.writeUTF(p.reason);
            out.writeInt(p.matches.size());
            for (var m : p.matches) {
              out.writeInt(m.a);
              out.writeInt(m.b);
            }
          }
          out.writeInt(features.tracks.size());
          for (var t : features.tracks) {
            out.writeInt(t.id);
            out.writeInt(t.observations.size());
            for (var ob : t.observations) {
              out.writeInt(ob.image);
              out.writeInt(ob.feature);
            }
          }
        });
    return false;
  }

  static CensusSgm.Result stereo(
      Options o,
      byte[] a,
      byte[] b,
      byte[] ca,
      byte[] cb,
      int w,
      int h,
      byte[] ma,
      byte[] mb,
      CensusSgm.Observer observer,
      Path artifacts)
      throws IOException {
    if (o.cacheDirectory == null) {
      CensusSgm.Result result = DenseMatcher.compute(o, a, b, ca, cb, w, h, ma, mb, observer);
      PointConfidence.stereo(result, a, b, o);
      return result;
    }
    String key =
        key(
            w
                + ":"
                + h
                + ":"
                + bytes(
                    a,
                    b,
                    ma,
                    mb,
                    o.censusMode.equals("color-distance") ? ca : null,
                    o.censusMode.equals("color-distance") ? cb : null)
                + ":bidirectional="
                + PointConfidence.bidirectional(o)
                + settings(
                    o,
                    "minDisparity",
                    "compute",
                    "gpuSgm",
                    "gpuAggregate",
                    "gpuDevice",
                    "memoryMb",
                    "gpuMemoryMb",
                    "gpuBankMb",
                    "gpuStripRows",
                    "esgmCandidates",
                    "disparities",
                    "census",
                    "censusMode",
                    "matchingCost",
                    "censusWeight",
                    "adTruncation",
                    "gradientTruncation",
                    "subpixelRefinement",
                    "median",
                    "medianMode",
                    "medianMaxChange",
                    "sgmEdgeScale",
                    "medianEdgeThreshold",
                    "speckleConnectivity",
                    "p1",
                    "p2",
                    "directions",
                    "uniqueness",
                    "correlation",
                    "correlationWindow",
                    "lrThreshold",
                    "speckleSize",
                    "speckleRange",
                    "depthOutlierFilter",
                    "depthOutlierRadius",
                    "depthOutlierDeviation",
                    "depthOutlierEdgeThreshold",
                    "pyramidLevels",
                    "pyramidRadius",
                    "tileSize",
                    "tileOverlap")
                // Appended only for tsgm so existing full-search cache keys are unchanged.
                + (o.pairwiseSearch.equals("full")
                    ? ""
                    : ":search="
                        + o.pairwiseSearch
                        + ":levels="
                        + o.tsgmLevels
                        + ":margin="
                        + o.tsgmMargin
                        + ":seeds="
                        + (o.tsgmSeeds == null ? 0 : Arrays.hashCode(o.tsgmSeeds)))
                // Opt-in depth-edge settings, appended only when used (keys otherwise unchanged).
                + (o.censusEdgeMask == 0 && o.censusShift == 0 && o.disparityEdgeErosion == 0
                    ? ""
                    : ":edges="
                        + o.censusEdgeMask
                        + ","
                        + o.censusShift
                        + ","
                        + o.disparityEdgeErosion
                        + ","
                        + o.disparityEdgeJump));
    CensusSgm.Result[] holder = new CensusSgm.Result[1];
    if (read(
        o,
        "stereo",
        key,
        artifacts,
        in -> {
          int rw = in.readInt(), rh = in.readInt();
          if (rw != w || rh != h) throw new IOException("Stereo cache dimension mismatch");
          CensusSgm.Stats stats = new CensusSgm.Stats();
          try {
            for (String name : STAT_NAMES)
              CensusSgm.Stats.class.getField(name).setLong(stats, in.readLong());
          } catch (ReflectiveOperationException e) {
            throw new IOException(e);
          }
          holder[0] =
              new CensusSgm.Result(
                  w, h, floats(in, w * h), stats, floats(in, w * h), floats(in, w * h));
          holder[0].confidence = floats(in, w * h);
          holder[0].costMargin = floats(in, w * h);
          holder[0].costSharpness = floats(in, w * h);
          if (PointConfidence.bidirectional(o)) holder[0].rightConfidence = floats(in, w * h);
          if (DepthOutlierFilter.enabled(o)) {
            holder[0].stats.depthOutlierRejected=in.readLong();
            holder[0].stats.depthOutlierReasons=new long[DepthOutlierFilter.REASONS.length];
            for(int i=0;i<DepthOutlierFilter.REASONS.length;i++)holder[0].stats.depthOutlierReasons[i]=in.readLong();
          }
        })) {
      var diagnostics=new DepthOutlierFilter.Diagnostics();
      diagnostics.add(holder[0].stats.depthOutlierReasons);
      DepthOutlierFilter.report(o,holder[0].stats.depthOutlierRejected,diagnostics);
      return holder[0];
    }
    CensusSgm.Result r = DenseMatcher.compute(o, a, b, ca, cb, w, h, ma, mb, observer);
    PointConfidence.stereo(r, a, b, o);
    write(
        o,
        "stereo",
        key,
        artifacts,
        out -> {
          out.writeInt(w);
          out.writeInt(h);
          try {
            for (String name : STAT_NAMES)
              out.writeLong(CensusSgm.Stats.class.getField(name).getLong(r.stats));
          } catch (ReflectiveOperationException e) {
            throw new IOException(e);
          }
          floats(out, r.disparity);
          floats(out, r.correlation);
          floats(out, r.leftRightError);
          floats(out, r.confidence);
          floats(out, r.costMargin);
          floats(out, r.costSharpness);
          if (PointConfidence.bidirectional(o)) floats(out, r.rightConfidence);
          if (DepthOutlierFilter.enabled(o)) {
            out.writeLong(r.stats.depthOutlierRejected);
            for(long count:r.stats.depthOutlierReasons)out.writeLong(count);
          }
        });
    return r;
  }

  static PatchMatch.Map patchmatch(Options o, PatchMatch.Scene scene, PatchMatch.Map previous)
      throws IOException {
    if (o.cacheDirectory == null) return PatchMatch.compute(scene, o, previous);
    MessageDigest hash = digest();
    hash.update(scene.pixels());
    hash.update(scene.masks());
    java.nio.ByteBuffer data = java.nio.ByteBuffer.allocate(scene.cameras().length * 4);
    for (float x : scene.cameras()) data.putFloat(x);
    hash.update(data.array());
    if (scene.geometry() != null) {
      java.nio.ByteBuffer block = java.nio.ByteBuffer.allocate(65536);
      for (float x : scene.geometry()) {
        if (block.remaining() < 4) {
          hash.update(block.array(), 0, block.position());
          block.clear();
        }
        block.putFloat(x);
      }
      hash.update(block.array(), 0, block.position());
    }
    if (previous != null) {
      java.nio.ByteBuffer block = java.nio.ByteBuffer.allocate(65536);
      for (float x : previous.data()) {
        if (block.remaining() < 4) {
          hash.update(block.array(), 0, block.position());
          block.clear();
        }
        block.putFloat(x);
      }
      hash.update(block.array(), 0, block.position());
    }
    String key =
        key(
            "patchmatch:"
                + PatchMatch.MODEL
                + ":"
                + scene.edgeSigma()
                + ":"
                + scene.geometricRelative()
                + ":"
                + (scene.geometry() == null ? "photo" : "geometric")
                + ":"
                + scene.width()
                + ":"
                + scene.height()
                + ":"
                + scene.minimum()
                + ":"
                + scene.maximum()
                + ":"
                + (previous == null ? "none" : previous.width() + "x" + previous.height())
                + ":"
                + HexFormat.of().formatHex(hash.digest())
                + settings(
                    o,
                    "denseMethod",
                    "compute",
                    "gpuDevice",
                    "patchmatchIterations",
                    "patchmatchViews",
                    "patchmatchLevels",
                    "patchmatchEdgeSigma",
                    "correlation",
                    "fusionRelative"));
    PatchMatch.Map[] cached = new PatchMatch.Map[1];
    if (read(
        o,
        "stereo",
        key,
        null,
        in -> {
          String producer = in.readUTF();
          float[] pixels =
              floats(in, Math.multiplyExact(Math.multiplyExact(scene.width(), scene.height()), 8));
          cached[0] =
              new PatchMatch.Map(
                  scene.width(),
                  scene.height(),
                  pixels,
                  Json.object("backend", "cache", "producer_backend", producer));
        })) return cached[0];
    PatchMatch.Map result = PatchMatch.compute(scene, o, previous);
    write(
        o,
        "stereo",
        key,
        null,
        out -> {
          out.writeUTF(Json.string(result.backend()));
          floats(out, result.data());
        });
    return result;
  }

  private static final String[] STAT_NAMES = {
    "lowerBoundaryRejected",
    "fullIntervalPixels",
    "coarseLowerBoundaryRejected",
    "coarseUpperBoundaryRejected",
    "coarseFullIntervalPixels",
    "boundaryRejected",
    "upperBoundaryRejected",
    "ambiguityRejected",
    "selected",
    "lrRejected",
    "postMedianLrRejected",
    "photometricRejected",
    "speckleRejected",
    "valid"
  };

  private static void floats(DataOutputStream out, float[] a) throws IOException {
    out.writeInt(a == null ? -1 : a.length);
    if (a != null) for (float v : a) out.writeFloat(v);
  }

  private static float[] floats(DataInputStream in, int expected) throws IOException {
    int n = in.readInt();
    if (n == -1) return null;
    if (n != expected) throw new IOException("Invalid raster length");
    float[] a = new float[n];
    for (int i = 0; i < n; i++) a[i] = in.readFloat();
    return a;
  }

  static String geometryKey(List<ImageFrame> frames, FeatureTracks features, Options o)
      throws IOException {

    StringBuilder b =
        new StringBuilder(
            matchesKey(frames, o)
                + settings(
                    o,
                    "iterations",
                    "baseline",
                    "seedPair",
                    "cleanupPasses",
                    "trackMinViews",
                    "trackMaxError",
                    "trackMinAngle",
                    "extrinsicsMode",
                    "positionSigma",
                    "rotationSigma",
                    "intrinsicsMode",
                    "refineIntrinsics",
                    "selfCalibration",
                    "focalSigma",
                    "radialSigma"));
    if (o.poses != null) b.append(hash(o.poses));
    for (var e : o.calibrationPoses.entrySet())
      b.append(e.getKey())
          .append(Arrays.toString(e.getValue().rotation))
          .append(Arrays.toString(e.getValue().translation));
    return key(b.toString());
  }

  static Calibration calibration(List<Path> paths, Options o, Calibration save) throws IOException {
    if (o.cacheDirectory == null) return save;
    StringBuilder b =
        new StringBuilder(
            settings(
                o,
                "focal",
                "sensorWidth",
                "selfCalibration",
                "calibrationPipeline",
                "matchingAlgorithm",
                "descriptorMatcher",
                "features",
                "ratio",
                "reprojection",
                "minMatches",
                "neighbors",
                "pairSelection",
                "pairCandidates",
                "iterations",
                "baseline",
                "intrinsicsMode",
                "extrinsicsMode",
                "refineIntrinsics",
                "focalSigma",
                "radialSigma",
                "positionSigma",
                "rotationSigma",
                "cleanupPasses",
                "trackMinViews",
                "trackMaxError",
                "trackMinAngle",
                "rectified"));
    b.append(featureSettings(o));
    for (Path p : paths) {
      int[] source = ResolutionPlan.originalSize(p),
          alignment = o.alignmentOptions().processedSize(source[0], source[1]);
      b.append(p.getFileName()).append(hash(p)).append(Arrays.toString(alignment));
    }
    if (o.calibration != null) b.append(hash(o.calibration));
    if (o.poses != null) b.append(hash(o.poses));
    String key = key(b.toString());
    Path artifacts = o.output.resolve("01_calibration");
    if (save == null) {
      Calibration[] c = new Calibration[1];
      boolean loaded =
          read(
              o,
              "calibration",
              key,
              artifacts,
              in -> {
                c[0] =
                    new Calibration(
                        in.readDouble(),
                        in.readDouble(),
                        in.readDouble(),
                        in.readDouble(),
                        vector(in),
                        in.readInt(),
                        in.readInt());
                Map<String, Camera> poses = new LinkedHashMap<>();
                int n = in.readInt();
                if (n < 0 || n > paths.size()) throw new IOException("Invalid calibration poses");
                for (int i = 0; i < n; i++) {
                  String name = in.readUTF();
                  Camera camera = new Camera(1, 1, 0, 0);
                  camera.rotation = vector(in);
                  camera.translation = vector(in);
                  camera.registered = true;
                  poses.put(name, camera);
                }
                o.calibrationPoses = poses;
              });
      return loaded ? c[0] : null;
    }
    write(
        o,
        "calibration",
        key,
        artifacts,
        out -> {
          out.writeDouble(save.fx);
          out.writeDouble(save.fy);
          out.writeDouble(save.cx);
          out.writeDouble(save.cy);
          vector(out, save.distortion);
          out.writeInt(save.width);
          out.writeInt(save.height);
          out.writeInt(o.calibrationPoses.size());
          for (var e : o.calibrationPoses.entrySet()) {
            out.writeUTF(e.getKey());
            vector(out, e.getValue().rotation);
            vector(out, e.getValue().translation);
          }
        });
    return save;
  }

  static boolean geometry(
      List<ImageFrame> frames, FeatureTracks features, Options o, String key, boolean save)
      throws IOException {
    if (o.cacheDirectory == null) return false;
    Path artifacts = o.output.resolve("04_geometry");
    if (!save)
      return read(
          o,
          "geometry",
          key,
          artifacts,
          in -> {
            if (in.readInt() != frames.size()) throw new IOException("Camera count mismatch");
            for (ImageFrame f : frames) {
              f.camera.registered = in.readBoolean();
              f.camera.fixed = in.readBoolean();
              f.camera.rotation = vector(in);
              f.camera.translation = vector(in);
              f.applyCalibration(
                  new Calibration(
                      in.readDouble(),
                      in.readDouble(),
                      in.readDouble(),
                      in.readDouble(),
                      vector(in),
                      in.readInt(),
                      in.readInt()));
            }
            int a = in.readInt(), b = in.readInt();
            features.seed = null;
            for (var p : features.pairs) if (p.a == a && p.b == b) features.seed = p;
            if (in.readInt() != features.tracks.size())
              throw new IOException("Track count mismatch");
            for (var t : features.tracks) {
              t.point = vector(in);
              t.observations.clear();
              int n = in.readInt();
              for (int i = 0; i < n; i++)
                t.observations.add(new FeatureTracks.Observation(in.readInt(), in.readInt()));
            }
          });
    write(
        o,
        "geometry",
        key,
        artifacts,
        out -> {
          out.writeInt(frames.size());
          for (ImageFrame f : frames) {
            out.writeBoolean(f.camera.registered);
            out.writeBoolean(f.camera.fixed);
            vector(out, f.camera.rotation);
            vector(out, f.camera.translation);
            Calibration c = f.calibration;
            out.writeDouble(c.fx);
            out.writeDouble(c.fy);
            out.writeDouble(c.cx);
            out.writeDouble(c.cy);
            vector(out, c.distortion);
            out.writeInt(c.width);
            out.writeInt(c.height);
          }
          out.writeInt(features.seed == null ? -1 : features.seed.a);
          out.writeInt(features.seed == null ? -1 : features.seed.b);
          out.writeInt(features.tracks.size());
          for (var t : features.tracks) {
            vector(out, t.point);
            out.writeInt(t.observations.size());
            for (var ob : t.observations) {
              out.writeInt(ob.image);
              out.writeInt(ob.feature);
            }
          }
        });
    return false;
  }
}
