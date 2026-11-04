package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;

/** Bounded LAS sampling and bidirectional model disagreement; the reference is not ground truth. */
final class ForestRevisionBenchmark {
  private record Points(double[] x, double[] y, double[] z, int[] order) {
    int size() {
      return x.length;
    }
  }

  public static void main(String[] args) throws Exception {
    Points reference = read(Path.of(args[0]), 500000);
    build(reference, 0, reference.size(), 0);
    List<Object> comparisons = new ArrayList<>();
    for (int i = 2; i < args.length; i++) {
      Path file = Path.of(args[i]);
      Points cloud = read(file, 4000000);
      Map<String, Object> forward = distances(cloud, reference, 150000);
      build(cloud, 0, cloud.size(), 0);
      Map<String, Object> reverse = distances(reference, cloud, reference.size());
      var result =
          Json.object(
              "file",
              file.toString(),
              "indexed_cloud_points",
              cloud.size(),
              "forward",
              forward,
              "reverse",
              reverse);
      comparisons.add(result);
      System.out.println(Json.string(result));
    }
    Json.write(
        Path.of(args[1]),
        Json.object(
            "reference",
            args[0],
            "reference_samples",
            reference.size(),
            "scope",
            "Bidirectional nearest-sample distances, clipped at 20 metres, in saved common"
                + " coordinates. Model disagreement and sampling density, not surveyed error.",
            "comparisons",
            comparisons));
  }

  private static Points read(Path file, int cap) throws IOException {
    try (FileChannel in = FileChannel.open(file)) {
      ByteBuffer header = ByteBuffer.allocate(375).order(ByteOrder.LITTLE_ENDIAN);
      while (header.hasRemaining() && in.read(header) > 0) {}
      if (header.get(0) != 'L'
          || header.get(1) != 'A'
          || header.get(2) != 'S'
          || header.get(3) != 'F') throw new IOException("Not LAS");
      if ((header.get(104) & 128) != 0) throw new IOException("Compressed LAS unsupported");
      int stride = Short.toUnsignedInt(header.getShort(105));
      long count = Integer.toUnsignedLong(header.getInt(107));
      if (count == 0 && Short.toUnsignedInt(header.getShort(94)) >= 375)
        count = header.getLong(247);
      int step = Math.max(1, (int) ((count + cap - 1) / cap)),
          n = (int) ((count + step - 1) / step);
      double[] x = new double[n], y = new double[n], z = new double[n];
      int[] order = new int[n];
      double sx = header.getDouble(131),
          sy = header.getDouble(139),
          sz = header.getDouble(147),
          ox = header.getDouble(155),
          oy = header.getDouble(163),
          oz = header.getDouble(171);
      in.position(Integer.toUnsignedLong(header.getInt(96)));
      ByteBuffer bytes = ByteBuffer.allocateDirect(stride * 8192).order(ByteOrder.LITTLE_ENDIAN);
      int at = 0;
      for (long first = 0; first < count; first += 8192) {
        int size = (int) Math.min(8192, count - first);
        bytes.clear();
        bytes.limit(size * stride);
        while (bytes.hasRemaining()) if (in.read(bytes) < 0) throw new EOFException();
        for (int j = 0; j < size; j++)
          if ((first + j) % step == 0) {
            int p = j * stride;
            x[at] = bytes.getInt(p) * sx + ox;
            y[at] = bytes.getInt(p + 4) * sy + oy;
            z[at] = bytes.getInt(p + 8) * sz + oz;
            order[at] = at;
            at++;
          }
      }
      return new Points(x, y, z, order);
    }
  }

  private static double coordinate(Points p, int id, int axis) {
    return axis == 0 ? p.x[id] : axis == 1 ? p.y[id] : p.z[id];
  }

  private static void build(Points p, int lo, int hi, int axis) {
    if (hi - lo <= 1) return;
    int mid = (lo + hi) >>> 1, a = lo, b = hi - 1;
    while (a < b) {
      double pivot = coordinate(p, p.order[(a + b) >>> 1], axis);
      int i = a, j = b;
      while (i <= j) {
        while (coordinate(p, p.order[i], axis) < pivot) i++;
        while (coordinate(p, p.order[j], axis) > pivot) j--;
        if (i <= j) {
          int t = p.order[i];
          p.order[i++] = p.order[j];
          p.order[j--] = t;
        }
      }
      if (mid <= j) b = j;
      else if (mid >= i) a = i;
      else break;
    }
    build(p, lo, mid, (axis + 1) % 3);
    build(p, mid + 1, hi, (axis + 1) % 3);
  }

  private static double nearest(
      Points p, int lo, int hi, int axis, double x, double y, double z, double best) {
    if (lo >= hi) return best;
    int mid = (lo + hi) >>> 1, id = p.order[mid];
    double dx = x - p.x[id], dy = y - p.y[id], dz = z - p.z[id];
    best = Math.min(best, dx * dx + dy * dy + dz * dz);
    double delta = axis == 0 ? dx : axis == 1 ? dy : dz;
    int next = (axis + 1) % 3;
    if (delta < 0) {
      best = nearest(p, lo, mid, next, x, y, z, best);
      if (delta * delta < best) best = nearest(p, mid + 1, hi, next, x, y, z, best);
    } else {
      best = nearest(p, mid + 1, hi, next, x, y, z, best);
      if (delta * delta < best) best = nearest(p, lo, mid, next, x, y, z, best);
    }
    return best;
  }

  private static Map<String, Object> distances(Points from, Points to, int cap) {
    int step = Math.max(1, (from.size() + cap - 1) / cap),
        n = 0,
        c05 = 0,
        c1 = 0,
        c2 = 0,
        c5 = 0,
        c10 = 0,
        c20 = 0;
    double sum = 0, clipped2 = 0;
    for (int i = 0; i < from.size(); i += step) {
      double d = nearest(to, 0, to.size(), 0, from.x[i], from.y[i], from.z[i], 400);
      n++;
      if (d < .25) c05++;
      if (d < 1) c1++;
      if (d < 4) c2++;
      if (d < 25) c5++;
      if (d < 100) c10++;
      if (d < 400) c20++;
      sum += Math.sqrt(d);
      clipped2 += Math.min(2, Math.sqrt(d));
    }
    return Json.object(
        "samples",
        n,
        "within_0_5_m",
        c05 / (double) n,
        "within_1_m",
        c1 / (double) n,
        "within_2_m",
        c2 / (double) n,
        "within_5_m",
        c5 / (double) n,
        "within_10_m",
        c10 / (double) n,
        "within_20_m",
        c20 / (double) n,
        "mean_distance_clipped_2_m",
        clipped2 / n,
        "mean_distance_clipped_20_m",
        sum / n);
  }
}
