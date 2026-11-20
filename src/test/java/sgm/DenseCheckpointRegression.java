package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * --dense-checkpoint building blocks: exact state encoding, counter snapshots, the settings
 * fingerprint, option parsing and the hard-link safety of re-created map files. The end-to-end
 * interrupt/resume comparisons (byte-identical clouds) are real-data runs, see DenseCheckpoint.
 */
final class DenseCheckpointRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "dense-checkpoint-");
    System.out.println("PASS dense checkpoint: " + run(root) + " checks; " + root);
  }

  @SuppressWarnings("unchecked")
  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    // Exact encoding: integers stay integers, doubles keep every bit, arrays and nesting survive.
    Map<String, Object> state = new LinkedHashMap<>();
    state.put("int", 7);
    state.put("long", Long.MAX_VALUE - 3);
    state.put("double", Math.nextUp(0.1));
    state.put("nan", Double.NaN);
    state.put("text", "camera ä");
    state.put("null", null);
    state.put("longs", new long[] {1, -2, Long.MIN_VALUE});
    state.put("ints", new int[] {3, 4});
    state.put("doubles", new double[] {Double.NEGATIVE_INFINITY, -0.0});
    state.put("flags", new boolean[] {true, false});
    state.put("list", Arrays.asList(1, null, Json.object("a", 2L)));
    Path file = root.resolve("state.bin");
    DenseCheckpoint.Codec.write(file, state);
    Map<String, Object> back = DenseCheckpoint.Codec.read(file);
    check(back.get("int") instanceof Integer i && i == 7, "Integer kept");
    check((long) back.get("long") == Long.MAX_VALUE - 3, "Long exact");
    check(Double.doubleToRawLongBits((double) back.get("double")) == Double.doubleToRawLongBits(Math.nextUp(0.1)), "Double bits exact");
    check(Double.isNaN((double) back.get("nan")) && back.get("text").equals("camera ä") && back.containsKey("null")
        && back.get("null") == null, "NaN, text and null kept");
    check(Arrays.equals((long[]) back.get("longs"), new long[] {1, -2, Long.MIN_VALUE})
        && Arrays.equals((int[]) back.get("ints"), new int[] {3, 4})
        && Arrays.equals((boolean[]) back.get("flags"), new boolean[] {true, false})
        && Double.doubleToRawLongBits(((double[]) back.get("doubles"))[1]) == Double.doubleToRawLongBits(-0.0),
        "Primitive arrays exact");
    List<Object> list = (List<Object>) back.get("list");
    check(list.size() == 3 && list.get(1) == null && ((Map<String, Object>) list.get(2)).get("a").equals(2L),
        "Nested list and map");
    boolean refused = false;
    try {
      DenseCheckpoint.Codec.write(root.resolve("bad.bin"), Json.object("x", new Object()));
    } catch (java.io.IOException expected) {
      refused = true;
    }
    check(refused, "Unknown objects are refused, not written as text");

    // Counter snapshots: primitive fields and long[] (final arrays restored in place).
    ImageFusion.Counters counters = new ImageFusion.Counters();
    counters.emitted = 41;
    counters.edgeEnabled = true;
    counters.viewHistogram[3] = 9;
    counters.cameraEmitted = new long[] {5, 6};
    Map<String, Object> saved = DenseCheckpoint.fields(counters);
    DenseCheckpoint.Codec.write(file, saved);
    ImageFusion.Counters restored = new ImageFusion.Counters();
    long[] histogram = restored.viewHistogram;
    DenseCheckpoint.restore(restored, DenseCheckpoint.Codec.read(file));
    check(restored.emitted == 41 && restored.edgeEnabled && restored.viewHistogram == histogram
        && restored.viewHistogram[3] == 9 && Arrays.equals(restored.cameraEmitted, new long[] {5, 6})
        && restored.cameraPixels == null, "Counters restore exactly");
    check(!saved.containsKey("memory"), "Object fields are not snapshotted");

    // Fingerprint: same options and state give the same value; resources may change; settings not.
    Path run = Files.createDirectories(root.resolve("run"));
    Files.createDirectories(run.resolve("state"));
    Files.write(run.resolve("state/frames.bin"), new byte[] {1, 2, 3});
    String[] base = {"-i", ".", "-o", run.toString(), "--dense-method", "mvsgm", "--fusion-mode", "image"};
    Options a = Options.parse(base), b = Options.parse(base);
    a.output = b.output = run;
    check(a.denseCheckpoint.equals("auto"), "Checkpoints default to auto");
    check(DenseCheckpoint.fingerprint(a).equals(DenseCheckpoint.fingerprint(b)), "Fingerprint is repeatable");
    Options resources = Options.parse(concat(base, "--cores", "3", "--memory-mb", "5000", "--mvsgm-cameras-in-flight", "2"));
    resources.output = run;
    check(DenseCheckpoint.fingerprint(resources).equals(DenseCheckpoint.fingerprint(a)), "Resources may change on resume");
    Options setting = Options.parse(concat(base, "--mvsgm-neighbors", "5"));
    setting.output = run;
    check(!DenseCheckpoint.fingerprint(setting).equals(DenseCheckpoint.fingerprint(a)), "Package-private mvsgm settings count");
    b.denseCheckpoint = "45";
    check(DenseCheckpoint.fingerprint(b).equals(DenseCheckpoint.fingerprint(a)), "The interval itself may change");
    var parts = DenseCheckpoint.fingerprintParts(a);
    Files.write(run.resolve("state/frames.bin"), new byte[] {1, 2, 4});
    List<String> changed = DenseCheckpoint.changed(new TreeMap<>(parts), DenseCheckpoint.fingerprintParts(a));
    check(changed.equals(List.of("state frames.bin")), "A changed earlier step is named: " + changed);

    // Option parsing and support.
    check(Options.parse(concat(base, "--dense-checkpoint", "OFF")).denseCheckpoint.equals("off"), "off parses");
    check(Options.parse(concat(base, "--dense-checkpoint", "15")).denseCheckpoint.equals("15"), "Minutes parse");
    boolean bad = false;
    try {
      Options.parse(concat(base, "--dense-checkpoint", "0"));
    } catch (IllegalArgumentException expected) {
      bad = true;
    }
    check(bad, "Zero minutes are refused");
    check(DenseCheckpoint.unsupported(a) == null, "mvsgm image fusion is supported");
    check(DenseCheckpoint.unsupported(Options.parse(concat(base, "--fusion-seed-order", "precision"))) != null,
        "Precision tiers are not");
    check(DenseCheckpoint.unsupported(Options.parse(new String[] {"-i", ".", "--dense-method", "sgm"})) != null,
        "Pair SGM is not");
    check(GuiOptions.all().stream().anyMatch(f -> f.flag().equals("--dense-checkpoint")), "GUI field");

    // A re-created map file never truncates a hard link that a checkpoint keeps.
    Path mapFile = root.resolve("camera_0.z"), kept = root.resolve("kept.z");
    try (MappedFiles mapped = new MappedFiles()) {
      var segment = mapped.create(mapFile, 64);
      segment.set(MappedFiles.FLOAT, 0, 3.5f);
      segment.force();
    }
    DenseCheckpoint.linkOrCopy(mapFile, kept);
    try (MappedFiles mapped = new MappedFiles()) {
      mapped.create(mapFile, 64);
    }
    check(Files.size(kept) == 64 && java.nio.ByteBuffer.wrap(Files.readAllBytes(kept))
        .order(java.nio.ByteOrder.LITTLE_ENDIAN).getFloat(0) == 3.5f, "Linked checkpoint map unchanged");
    DenseCheckpoint.deleteTree(root);
    return checks;
  }

  private static String[] concat(String[] a, String... b) {
    String[] out = Arrays.copyOf(a, a.length + b.length);
    System.arraycopy(b, 0, out, a.length, b.length);
    return out;
  }
}
