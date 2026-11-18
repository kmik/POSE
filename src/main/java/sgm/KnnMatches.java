package sgm;

import java.util.*;
import org.opencv.core.*;

/** Two-neighbour results without one finalizable native matrix per query. */
final class KnnMatches {
  private int rows;
  private byte[] counts = new byte[0];
  // Same CV_32FC4 representation as OpenCV's MatOfDMatch: query, train, image, distance.
  private float[] values = new float[0];

  int size() { return rows; }
  int count(int row) { return counts[row]; }
  int query(int row) { return (int) values[8 * row]; }
  int train(int row) { return (int) values[8 * row + 1]; }
  float distance(int row) { return values[8 * row + 3]; }
  float secondDistance(int row) { return values[8 * row + 7]; }

  void reset(int size) {
    if (counts.length < size) counts = new byte[size];
    if (values.length < Math.multiplyExact(size, 8)) values = new float[Math.multiplyExact(size, 8)];
    Arrays.fill(counts, 0, size, (byte) 0);
    rows = size;
  }

  void set(int row, int count, float[] data) {
    counts[row] = (byte) count;
    System.arraycopy(data, 0, values, 8 * row, 4 * count);
  }

  void setGpu(int row, int first, int firstDistance, int second, int secondDistance) {
    int at = 8 * row;
    counts[row] = (byte) (second >= 0 ? 2 : first >= 0 ? 1 : 0);
    values[at] = row; values[at + 1] = first; values[at + 2] = 0; values[at + 3] = firstDistance;
    values[at + 4] = row; values[at + 5] = second; values[at + 6] = 0; values[at + 7] = secondDistance;
  }

  void copyFrom(KnnMatches other) {
    reset(other.rows);
    System.arraycopy(other.counts, 0, counts, 0, rows);
    System.arraycopy(other.values, 0, values, 0, 8 * rows);
  }

  long bytes() { return 33L * rows + 64; }

  DMatch[] rowArray(int row) {
    DMatch[] result = new DMatch[count(row)];
    for (int k = 0; k < result.length; k++) {
      int at = 8 * row + 4 * k;
      result[k] = new DMatch((int) values[at], (int) values[at + 1], (int) values[at + 2], values[at + 3]);
    }
    return result;
  }

  // Compatibility for older callers/tests only. Production matching consumes the flat form.
  void toLegacy(List<MatOfDMatch> out) {
    for (int i = 0; i < rows; i++) out.add(new MatOfDMatch(rowArray(i)));
  }
}
