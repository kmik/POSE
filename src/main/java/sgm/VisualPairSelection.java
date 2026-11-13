package sgm;

import java.io.*;
import java.lang.ref.Reference;
import java.util.*;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.imgproc.Imgproc;

/** One compact preview per image, one global retrieval index, then exact reciprocal scoring. */
final class VisualPairSelection {
  private static final int LIMIT = 256, GRID = 8;
  record Result(String search, Map<Long, Double> scores) {}
  record Preview(Mat descriptors, int[] cells) implements AutoCloseable {
    public void close() { descriptors.release(); }
  }

  static Result scores(List<ImageFrame> frames, Options o) throws IOException {
    // FLANN uses a thread-local RNG. Keep retrieval repeatable without altering serial matching.
    CompletableFuture<Result> task = new CompletableFuture<>();
    Thread worker = new Thread(() -> {
      try { Core.setRNGSeed(1597463007); task.complete(compute(frames, o)); }
      catch (Throwable e) { task.completeExceptionally(e); }
    }, "visual-pair-selection");
    worker.setDaemon(true);
    worker.start();
    try { return task.join(); }
    catch (CompletionException e) {
      Throwable cause = e.getCause();
      if (cause instanceof IOException io) throw io;
      if (cause instanceof RuntimeException r) throw r;
      if (cause instanceof Error error) throw error;
      throw new IOException(cause);
    }
  }

  private static Result compute(List<ImageFrame> frames, Options o) throws IOException {
    int n = frames.size();
    int breadth = (int) Math.min(Math.max(0, n - 1), (long) o.neighbors + o.pairCandidates);
    List<Preview> previews = new ArrayList<>();
    try (WorkProgress work = WorkProgress.start("Estimate visual overlap", n, "images")) {
      long memory = 0;
      for (ImageFrame f : frames) {
        Preview p = preview(f, o);
        previews.add(p);
        work.update(previews.size());
        // Raw previews plus converted training data and FLANN's retained index/merge copies.
        memory += p.descriptors.total() * (p.descriptors.depth() == CvType.CV_8U ? 161L : 24L);
        if (memory > o.memoryMb * 1048576L / 4)
          throw new IOException("Visual pair index exceeds one quarter of --memory-mb; increase it"
              + " or use --pair-selection gps/sequential explicitly");
      }
      Set<Long> candidates = new TreeSet<>();
      work.detail("retrieving candidate images");
      boolean exhaustive = breadth >= n - 1;
      if (exhaustive) {
        for (int a = 0; a < n; a++) for (int b = a + 1; b < n; b++)
          candidates.add(PairSelection.key(a, b));
      } else retrieve(previews, breadth, candidates);
      Map<Long, Double> scores = new LinkedHashMap<>();
      work.detail("scoring " + candidates.size() + " preview pairs");
      for (long edge : candidates) {
        int a = (int) (edge >>> 32), b = (int) edge;
        scores.put(edge, score(previews.get(a), previews.get(b), o.ratio));
      }
      work.done();
      return new Result(exhaustive ? "exhaustive_descriptor_preview" : "indexed_descriptor_preview", scores);
    } finally { for (Preview p : previews) p.close(); }
  }

  private static Preview preview(ImageFrame f, Options o) throws IOException {
    if (!f.descriptors.empty() || f.descriptorFile != null) {
      DescriptorCache.pair(f, f);
      return sample(f.descriptors, f.keypoints, f.width, f.height);
    }
    // Fixed-pose workflows can skip full feature extraction. Decode a bounded preview for them.
    double scale = Math.min(1, 640.0 / Math.max(f.sourceWidth, f.sourceHeight));
    int w = Math.max(1, (int) Math.round(f.sourceWidth * scale));
    int h = Math.max(1, (int) Math.round(f.sourceHeight * scale));
    Mat color = WindowedImage.read(f.path, f.sourceWidth, f.sourceHeight, w, h,
        o.imageWindow, 1, WindowedImage.wholeDecodeBytes(o));
    Mat gray = new Mat(), descriptors = new Mat(), mask = new Mat();
    MatOfKeyPoint points = new MatOfKeyPoint();
    Feature2D detector = FeatureAlgorithms.detector(o, LIMIT);
    try {
      Imgproc.cvtColor(color, gray, Imgproc.COLOR_BGR2GRAY);
      detector.detectAndCompute(gray, mask, points, descriptors);
      return sample(descriptors, points.toArray(), w, h);
    } finally {
      Reference.reachabilityFence(detector);
      color.release(); gray.release(); descriptors.release(); mask.release(); points.release();
    }
  }

  private static Preview sample(Mat source, KeyPoint[] points, double w, double h) {
    if (source.empty()) return new Preview(new Mat(), new int[0]);
    List<List<Integer>> bins = new ArrayList<>();
    for (int i = 0; i < GRID * GRID; i++) bins.add(new ArrayList<>());
    for (int row = 0; row < source.rows(); row++) bins.get(cell(points[row], w, h)).add(row);
    Comparator<Integer> strongest = Comparator.<Integer>comparingDouble(i -> points[i].response)
        .reversed().thenComparingInt(i -> i);
    for (var bin : bins) bin.sort(strongest);
    int count = Math.min(LIMIT, source.rows()), filled = 0;
    Mat out = new Mat(count, source.cols(), source.type());
    int[] cells = new int[count];
    try {
      for (int depth = 0; filled < count; depth++) for (int bin = 0; bin < bins.size(); bin++) {
        if (depth >= bins.get(bin).size()) continue;
        Mat row = source.row(bins.get(bin).get(depth)), target = out.row(filled);
        try { row.copyTo(target); } finally { row.release(); target.release(); }
        cells[filled++] = bin;
        if (filled == count) break;
      }
      return new Preview(out, cells);
    } catch (RuntimeException | Error e) { out.release(); throw e; }
  }

  private static int cell(KeyPoint p, double w, double h) {
    int x = Math.max(0, Math.min(GRID - 1, (int) (GRID * p.pt.x / w)));
    int y = Math.max(0, Math.min(GRID - 1, (int) (GRID * p.pt.y / h)));
    return y * GRID + x;
  }

  private static Mat indexed(Mat raw) {
    if (raw.depth() != CvType.CV_8U) return raw.clone();
    // Bit expansion makes squared L2 exactly Hamming before the approximate KD-tree search.
    byte[] bytes = new byte[(int) raw.total()];
    raw.get(0, 0, bytes);
    float[] bits = new float[bytes.length * 8];
    for (int i = 0; i < bytes.length; i++) for (int b = 0; b < 8; b++)
      bits[8 * i + b] = ((bytes[i] & 255) >>> b) & 1;
    Mat out = new Mat(raw.rows(), raw.cols() * 8, CvType.CV_32F);
    out.put(0, 0, bits);
    return out;
  }

  private static void retrieve(List<Preview> previews, int breadth, Set<Long> candidates) {
    List<Mat> training = new ArrayList<>();
    List<Integer> ids = new ArrayList<>();
    DescriptorMatcher index = DescriptorMatcher.create(DescriptorMatcher.FLANNBASED);
    try {
      long rows = 0;
      for (int a = 0; a < previews.size(); a++) if (!previews.get(a).descriptors.empty()) {
        Mat m = indexed(previews.get(a).descriptors);
        training.add(m); ids.add(a); rows += m.rows();
      }
      if (rows < 2) return;
      index.add(training); index.train();
      int nearest = (int) Math.min(rows, Math.max(16L, Math.min(128L, 4L * breadth)));
      for (int query = 0; query < training.size(); query++) {
        int a = ids.get(query);
        int[] votes = new int[previews.size()];
        List<MatOfDMatch> matches = new ArrayList<>();
        try {
          index.knnMatch(training.get(query), matches, nearest);
          for (MatOfDMatch row : matches) {
            BitSet seen = new BitSet(votes.length);
            for (DMatch m : row.toArray()) {
              int b = ids.get(m.imgIdx);
              if (b != a && !seen.get(b)) { votes[b]++; seen.set(b); }
            }
          }
        } finally { for (Mat m : matches) m.release(); }
        List<Integer> ranked = new ArrayList<>();
        for (int b = 0; b < votes.length; b++) if (votes[b] > 0) ranked.add(b);
        ranked.sort(Comparator.<Integer>comparingInt(b -> votes[b]).reversed().thenComparingInt(b -> b));
        for (int i = 0; i < Math.min(breadth, ranked.size()); i++)
          candidates.add(PairSelection.key(a, ranked.get(i)));
      }
    } finally { index.clear(); for (Mat m : training) m.release(); }
  }

  private static double score(Preview a, Preview b, double ratio) {
    Mat x = a.descriptors, y = b.descriptors;
    if (x.rows() < 2 || y.rows() < 2) return 0;
    DescriptorMatcher matcher = BFMatcher.create(x.depth() == CvType.CV_8U ? Core.NORM_HAMMING : Core.NORM_L2, false);
    List<MatOfDMatch> fw = new ArrayList<>(), bw = new ArrayList<>();
    try {
      matcher.knnMatch(x, y, fw, 2); matcher.knnMatch(y, x, bw, 2);
      int[] reverse = new int[y.rows()]; Arrays.fill(reverse, -1);
      for (var row : bw) {
        DMatch[] m = row.toArray();
        if (m.length >= 2 && m[0].distance < ratio * m[1].distance)
          reverse[m[0].queryIdx] = m[0].trainIdx;
      }
      int support = 0;
      BitSet ca = new BitSet(GRID * GRID), cb = new BitSet(GRID * GRID);
      for (var row : fw) {
        DMatch[] m = row.toArray();
        if (m.length >= 2 && m[0].distance < ratio * m[1].distance
            && reverse[m[0].trainIdx] == m[0].queryIdx) {
          support++; ca.set(a.cells[m[0].queryIdx]); cb.set(b.cells[m[0].trainIdx]);
        }
      }
      return support < 4 ? 0 : support / Math.sqrt((double) x.rows() * y.rows())
          * Math.sqrt((double) ca.cardinality() * cb.cardinality()) / (GRID * GRID);
    } finally {
      matcher.clear(); for (Mat m : fw) m.release(); for (Mat m : bw) m.release();
    }
  }
}
