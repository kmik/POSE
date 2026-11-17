package sgm;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;
import java.util.zip.CRC32;

/**
 * Pair matching checkpoint (03_matches/pair_checkpoint.bin): every verified pair is appended as
 * it is consumed, so a match step that is killed (out of memory, job time limit) and continued in
 * the same run (--run ID) does not search and verify the finished pairs again.
 *
 * <p>Only in parallel pair matching, where each pair's search and verification are seeded from its
 * own image ids and therefore a function of its two descriptor sets (PairMatching): a record is
 * keyed by both image ids and descriptor digests, and the file header by the matching settings, so
 * a restored pair is the pair the step would compute again. Records carry a length and a CRC-32;
 * a record cut by the kill is dropped and overwritten. The file is deleted when the step ends.
 * Not used with --diagnostics --storage debug, whose exports need the raw match lists.
 */
final class MatchCheckpoint implements Closeable {
  private static final int MAGIC = 0x504d4331; // "PMC1"

  /** A finished pair and the counts its compact export needs. */
  record Record(FeatureTracks.Pair pair, int queries, int ransac, int cheirality) {}

  private final Path file;
  private final FileChannel channel;
  private final Map<String, Long> index = new HashMap<>();
  private long end;
  int restored, written;

  private MatchCheckpoint(Path file, FileChannel channel) {
    this.file = file;
    this.channel = channel;
  }

  static boolean enabled(Options o, Project project) {
    return project != null
        && PairMatching.parallel(o)
        && !(o.diagnostics && o.storage.equals("debug"))
        && !"false".equals(System.getProperty("sgm.matchCheckpoint"));
  }

  static String configuration(Options o) {
    return ReuseCache.settings(
        o,
        "matchingAlgorithm",
        "descriptorMatcher",
        "ratio",
        "reprojection",
        "minMatches",
        "pairMatching",
        "compute",
        "siftContrast",
        "siftEdgeThreshold",
        "orbFastThreshold",
        "briskThreshold",
        "akazeThreshold",
        "kazeThreshold");
  }

  /** The step's checkpoint, with the records an earlier attempt left; null when not used. */
  static MatchCheckpoint open(Options o, Project project) throws IOException {
    return enabled(o, project) ? openUnchecked(o) : null;
  }

  /** The checkpoint of o.output, whatever the mode (tests). */
  static MatchCheckpoint openUnchecked(Options o) throws IOException {
    Path file = o.output.resolve("03_matches/pair_checkpoint.bin");
    Files.createDirectories(file.getParent());
    String config = configuration(o);
    FileChannel channel =
        FileChannel.open(file, StandardOpenOption.CREATE, StandardOpenOption.READ, StandardOpenOption.WRITE);
    MatchCheckpoint c = new MatchCheckpoint(file, channel);
    try {
      c.load(config);
    } catch (IOException | RuntimeException e) {
      channel.close();
      throw e;
    }
    if (!c.index.isEmpty())
      System.out.printf(
          Locale.ROOT, "Pair matching: %,d verified pairs from an earlier attempt of this step%n", c.index.size());
    return c;
  }

  private void load(String config) throws IOException {
    long size = channel.size();
    byte[] header = header(config);
    boolean valid = size >= header.length && Arrays.equals(read(0, header.length), header);
    if (!valid) {
      channel.truncate(0);
      write(0, header);
      end = header.length;
      return;
    }
    long at = header.length;
    while (at + 4 <= size) {
      int length = ByteBuffer.wrap(read(at, 4)).getInt();
      if (length <= 0 || at + 8 + length > size) break;
      byte[] payload = read(at + 4, length);
      int crc = ByteBuffer.wrap(read(at + 4 + length, 4)).getInt();
      CRC32 check = new CRC32();
      check.update(payload);
      if ((int) check.getValue() != crc) break;
      String key = new DataInputStream(new ByteArrayInputStream(payload)).readUTF();
      index.put(key, at);
      at += 8 + length;
    }
    end = at;
    channel.truncate(end); // a record cut by a kill is overwritten
  }

  private static byte[] header(String config) throws IOException {
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    DataOutputStream out = new DataOutputStream(bytes);
    out.writeInt(MAGIC);
    out.writeUTF(config.length() > 20000 ? ReuseCache.key(config) : config);
    out.flush();
    return bytes.toByteArray();
  }

  private byte[] read(long at, int n) throws IOException {
    ByteBuffer b = ByteBuffer.allocate(n);
    while (b.hasRemaining()) if (channel.read(b, at + b.position()) < 0) break;
    return Arrays.copyOf(b.array(), b.position());
  }

  private void write(long at, byte[] bytes) throws IOException {
    ByteBuffer b = ByteBuffer.wrap(bytes);
    while (b.hasRemaining()) at += channel.write(b, at);
  }

  private static String key(ImageFrame a, ImageFrame b) {
    if (a.descriptorDigest == null || b.descriptorDigest == null) return null;
    return a.id + ":" + b.id + ":" + a.descriptorDigest + ":" + b.descriptorDigest;
  }

  /** The finished pair of these frames, or null. */
  synchronized Record get(ImageFrame a, ImageFrame b) throws IOException {
    String key = key(a, b);
    Long at = key == null ? null : index.get(key);
    if (at == null) return null;
    int length = ByteBuffer.wrap(read(at, 4)).getInt();
    DataInputStream in = new DataInputStream(new ByteArrayInputStream(read(at + 4, length)));
    in.readUTF();
    int queries = in.readInt(), ransac = in.readInt(), cheirality = in.readInt();
    FeatureTracks.Pair pair = new FeatureTracks.Pair(a.id, b.id);
    pair.reason = in.readUTF();
    pair.rotation = vector(in);
    pair.translation = vector(in);
    pair.essential = vector(in);
    int n = in.readInt();
    for (int i = 0; i < n; i++) pair.matches.add(new FeatureTracks.Match(in.readInt(), in.readInt()));
    restored++;
    return new Record(pair, queries, ransac, cheirality);
  }

  /** Appends a pair as consumed (single consumer thread, in pair order). */
  synchronized void put(ImageFrame a, ImageFrame b, FeatureTracks.Pair pair, int queries, int ransac, int cheirality)
      throws IOException {
    String key = key(a, b);
    if (key == null || index.containsKey(key)) return;
    ByteArrayOutputStream bytes = new ByteArrayOutputStream(64 + 8 * pair.matches.size());
    DataOutputStream out = new DataOutputStream(bytes);
    out.writeUTF(key);
    out.writeInt(queries);
    out.writeInt(ransac);
    out.writeInt(cheirality);
    out.writeUTF(pair.reason == null ? "" : pair.reason);
    vector(out, pair.rotation);
    vector(out, pair.translation);
    vector(out, pair.essential);
    out.writeInt(pair.matches.size());
    for (FeatureTracks.Match m : pair.matches) {
      out.writeInt(m.a);
      out.writeInt(m.b);
    }
    out.flush();
    byte[] payload = bytes.toByteArray();
    CRC32 crc = new CRC32();
    crc.update(payload);
    ByteBuffer record = ByteBuffer.allocate(8 + payload.length);
    record.putInt(payload.length).put(payload).putInt((int) crc.getValue());
    write(end, record.array());
    index.put(key, end);
    end += record.capacity();
    written++;
  }

  private static void vector(DataOutputStream out, double[] v) throws IOException {
    if (v == null) {
      out.writeInt(-1);
      return;
    }
    out.writeInt(v.length);
    for (double x : v) out.writeDouble(x);
  }

  private static double[] vector(DataInputStream in) throws IOException {
    int n = in.readInt();
    if (n < 0) return null;
    double[] v = new double[n];
    for (int i = 0; i < n; i++) v[i] = in.readDouble();
    return v;
  }

  /** Closes; with {@code finished} the file is deleted (the step's results are saved elsewhere). */
  void close(boolean finished) throws IOException {
    channel.close();
    if (finished) Files.deleteIfExists(file);
  }

  @Override
  public void close() throws IOException {
    close(false);
  }
}
