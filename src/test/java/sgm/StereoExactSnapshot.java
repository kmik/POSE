package sgm;

import java.io.*;
import java.lang.reflect.Field;
import java.nio.file.*;
import java.security.*;
import java.util.*;

/** The same harness runs against original and revised classes; snapshots include all filter stages. */
public final class StereoExactSnapshot {
  private static void floats(DataOutputStream out, float[] values) throws IOException {
    out.writeInt(values == null ? -1 : values.length);
    if (values != null) for (float value : values) out.writeInt(Float.floatToRawIntBits(value));
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    org.opencv.core.Core.setNumThreads(1);
    Path output = Path.of(args[0]);
    Files.createDirectories(output.toAbsolutePath().getParent());
    int cores = Integer.parseInt(args[1]), cases = 0;
    int w = 73, h = 49, n = w * h;
    long started = System.nanoTime(), valid = 0;
    try (var out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(output)))) {
      for (String cost : new String[] {"census", "ad-census", "census-gradient", "zncc"})
        for (int window : new int[] {5, 9, 11})
          for (String mode : new String[] {"gray", "color-distance"})
            for (int paths : new int[] {4, 8}) {
              Options o = new Options();
              o.output = output.toAbsolutePath().getParent();
              o.compute = "cpu";
              o.cores = cores;
              o.matchingCost = cost;
              o.census = window;
              o.censusMode = mode;
              o.directions = paths;
              o.disparities = 13;
              o.minDisparity = new int[] {-3, 0, 3}[cases % 3];
              o.sgmAggregate = cases % 2 == 0 ? "16" : "32";
              o.sgmEdgeScale = cases % 3 == 0 ? 0 : 32;
              o.medianMode = cases % 3 == 0 ? "classic" : "edge-preserving";
              o.subpixelRefinement = cases % 12 == 0 ? "zncc" : "none";
              o.twoViewConfidenceThreshold = .75;
              o.memoryMb = 256;
              byte[] a = new byte[n], b = new byte[n], ca = new byte[n * 3], cb = new byte[n * 3];
              byte[] ma = new byte[n], mb = new byte[n];
              Random random = new Random(7717 + cases);
              random.nextBytes(a);
              random.nextBytes(ca);
              for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
                int p = y * w + x, shift = o.minDisparity + 4 + (y >= h / 2 ? 1 : 0);
                int q = y * w + Math.max(0, Math.min(w - 1, x + shift));
                b[p] = a[q];
                System.arraycopy(ca, q * 3, cb, p * 3, 3);
                ma[p] = (byte) (p % 29 == 0 ? 0 : 255);
                mb[p] = (byte) (p % 31 == 0 ? 0 : 255);
              }
              out.writeUTF(cost + "/" + window + "/" + mode + "/" + paths);
              CensusSgm.Observer observer = new CensusSgm.Observer() {
                public void raster(String name, float[] values, int width, int height) throws IOException {
                  out.writeUTF(name);
                  floats(out, values);
                }
                public void reasons(byte[] codes, int width, int height) throws IOException {
                  out.writeUTF("reasons"); out.writeInt(codes.length); out.write(codes);
                }
                public void searchBoundary(byte[] flags, int width, int height) throws IOException {
                  out.writeUTF("boundary"); out.writeInt(flags.length); out.write(flags);
                }
              };
              var result = new CensusSgm(o).computeRecorded(a, b, ca, cb, w, h, ma, mb, observer);
              PointConfidence.stereo(result, a, b);
              for (float[] values : new float[][] {result.disparity, result.correlation,
                  result.leftRightError, result.costMargin, result.costSharpness,
                  result.confidence, result.rightConfidence}) floats(out, values);
              Field[] fields = CensusSgm.Stats.class.getFields();
              Arrays.sort(fields, Comparator.comparing(Field::getName));
              for (Field field : fields) {
                out.writeUTF(field.getName());
                out.writeLong(field.getLong(result.stats));
              }
              valid += result.stats.valid;
              cases++;
            }
    }
    byte[] hash = MessageDigest.getInstance("SHA-256").digest(Files.readAllBytes(output));
    System.out.printf(Locale.ROOT, "SNAPSHOT cases=%d valid=%d cores=%d seconds=%.3f sha256=%s%n",
        cases, valid, cores, (System.nanoTime() - started) / 1e9, HexFormat.of().formatHex(hash));
  }
}
