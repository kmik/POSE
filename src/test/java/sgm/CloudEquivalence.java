package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.GZIPInputStream;

/** Compare ordered real-run clouds, credited support and accepted candidate IDs. */
final class CloudEquivalence {
  static ByteBuffer cloud(Path run) throws IOException {
    byte[] data = Files.readAllBytes(run.resolve("07_results/dense.ply"));
    byte[] end = "end_header\n".getBytes(java.nio.charset.StandardCharsets.US_ASCII);
    int first = -1;
    outer:
    for (int i = 0; i < Math.min(65536, data.length - end.length); i++) {
      for (int j = 0; j < end.length; j++) if (data[i + j] != end[j]) continue outer;
      first = i + end.length;
      break;
    }
    if (first < 0) throw new IOException("Unknown PLY layout");
    String header = new String(data, 0, first, java.nio.charset.StandardCharsets.US_ASCII);
    int stride = header.contains("property float nx") ? 59 : 27;
    if ((data.length - first) % stride != 0) throw new IOException("Invalid PLY length");
    ByteBuffer plain =
        ByteBuffer.allocate((data.length - first) / stride * 27).order(ByteOrder.LITTLE_ENDIAN);
    for (int at = first; at < data.length; at += stride) plain.put(data, at, 27);
    return plain.flip();
  }

  static BufferedReader confidence(Path run) throws IOException {
    if (!Files.exists(run.resolve("07_results/point-confidence.csv.gz"))) {
      var binary = new PointRecords.Reader(run.resolve("07_results/point-confidence.bin"));
      return new BufferedReader(new StringReader("")) {
        boolean header = true;

        @Override
        public String readLine() throws IOException {
          if (header) {
            header = false;
            return PointRecords.HEADER.stripTrailing();
          }
          var row = binary.next();
          return row == null ? null : row.csv().stripTrailing();
        }

        @Override
        public void close() throws IOException {
          try {
            binary.close();
          } finally {
            super.close();
          }
        }
      };
    }
    return new BufferedReader(
        new InputStreamReader(
            new GZIPInputStream(
                Files.newInputStream(run.resolve("07_results/point-confidence.csv.gz")), 65536),
            java.nio.charset.StandardCharsets.UTF_8));
  }

  public static void main(String[] args) throws Exception {
    Path a = Path.of(args[0]), b = Path.of(args[1]);
    ByteBuffer x = cloud(a), y = cloud(b);
    if (x.remaining() != y.remaining()) throw new AssertionError("Cloud point count differs");
    int count = x.remaining() / 27;
    double maximum = 0, sigma = 0, shift = 0;
    for (int i = 0; i < count; i++) {
      double square = 0;
      for (int j = 0; j < 3; j++) {
        double d = x.getDouble() - y.getDouble();
        square += d * d;
      }
      maximum = Math.max(maximum, Math.sqrt(square));
      for (int j = 0; j < 3; j++) if (x.get() != y.get()) throw new AssertionError("RGB differs");
    }
    if (maximum > 1e-8) throw new AssertionError("Point coordinate difference " + maximum);
    try (BufferedReader ca = confidence(a);
        BufferedReader cb = confidence(b)) {
      if (!Objects.equals(ca.readLine(), cb.readLine()))
        throw new AssertionError("Confidence schema differs");
      for (int i = 0; i < count; i++) {
        String[] p = ca.readLine().split(","), q = cb.readLine().split(",");
        for (int j : new int[] {0, 1, 2, 6, 7, 8, 9, 10})
          if (!p[j].equals(q[j]))
            throw new AssertionError(
                "Accepted candidate/support/order differs at point " + i + " column " + j);
        sigma = Math.max(sigma, Math.abs(Double.parseDouble(p[11]) - Double.parseDouble(q[11])));
        shift = Math.max(shift, Math.abs(Double.parseDouble(p[12]) - Double.parseDouble(q[12])));
      }
      if (ca.readLine() != null || cb.readLine() != null)
        throw new AssertionError("Extra confidence rows");
    }
    if (sigma > 1e-9 || shift > 1e-8) throw new AssertionError("Confidence difference");
    Map<String, Object> report =
        Json.object(
            "reference_run",
            a.toAbsolutePath().toString(),
            "compared_run",
            b.toAbsolutePath().toString(),
            "points",
            count,
            "max_xyz_error",
            maximum,
            "max_relative_sigma_error",
            sigma,
            "max_refinement_shift_error",
            shift,
            "same_accepted_candidates_support_and_order",
            true,
            "las_byte_identical",
            ReuseCache.hash(a.resolve("07_results/dense.las"))
                .equals(ReuseCache.hash(b.resolve("07_results/dense.las"))));
    if (args.length > 2) Json.write(Path.of(args[2]), report);
    System.out.println("PASS ordered cloud equivalence: " + Json.string(report));
  }
}
