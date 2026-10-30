package sgm;

import java.nio.*;
import java.nio.file.*;
import java.util.*;

/** Read-only comparison to a previous full-range disparity; agreement is not ground truth. */
final class DisparityComparison {
  private static float[] read(Path path) throws Exception {
    byte[] data = Files.readAllBytes(path);
    int offset = 0, lines = 0;
    while (lines < 3) if (data[offset++] == 10) lines++;
    float[] result = new float[(data.length - offset) / 4];
    ByteBuffer.wrap(data, offset, data.length - offset)
        .order(ByteOrder.LITTLE_ENDIAN)
        .asFloatBuffer()
        .get(result);
    return result;
  }

  public static void main(String[] args) throws Exception {
    float[] a = read(Path.of(args[0])), b = read(Path.of(args[1]));
    if (a.length != b.length) throw new IllegalArgumentException("Different raster sizes");
    long before = 0,
        after = 0,
        common = 0,
        lost = 0,
        added = 0,
        within01 = 0,
        within1 = 0,
        highBefore = 0,
        highKept = 0;
    double[] errors = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      boolean x = Float.isFinite(a[i]), y = Float.isFinite(b[i]);
      if (x) before++;
      if (y) after++;
      if (x && a[i] > 600) {
        highBefore++;
        if (y && Math.abs(a[i] - b[i]) < 1) highKept++;
      }
      if (x && y) {
        double e = Math.abs(a[i] - b[i]);
        errors[(int) common++] = e;
        if (e < .1) within01++;
        if (e < 1) within1++;
      } else if (x) lost++;
      else if (y) added++;
    }
    errors = Arrays.copyOf(errors, (int) common);
    Arrays.sort(errors);
    Object report =
        Json.object(
            "previous_valid",
            before,
            "new_valid",
            after,
            "common",
            common,
            "lost",
            lost,
            "added",
            added,
            "common_within_0_1_pixel",
            within01,
            "common_within_1_pixel",
            within1,
            "median_absolute_difference",
            common == 0 ? null : errors[(int) common / 2],
            "p99_absolute_difference",
            common == 0 ? null : errors[(int) ((common - 1) * .99)],
            "previous_above_600",
            highBefore,
            "above_600_kept_within_1_pixel",
            highKept,
            "scope",
            "comparison to prior full-range estimate, not independent ground truth");
    System.out.println(Json.string(report));
    if (args.length > 2) Json.write(Path.of(args[2]), report);
  }
}
