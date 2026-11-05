import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.*;

/** Tests packaging integrity and non-native JAR preservation, not photogrammetry algorithms. */
public class PortableReleaseRegression {
  private static int checks;
  private static void require(boolean value, String message) {
    if (!value) throw new AssertionError(message);
    checks++;
  }
  private static void fails(String text, String... args) throws Exception {
    try { PortableRelease.main(args); }
    catch (IOException | IllegalArgumentException failure) {
      require(failure.getMessage().contains(text), "Unexpected failure: " + failure);
      return;
    }
    throw new AssertionError("Expected rejection: " + Arrays.toString(args));
  }
  public static void main(String[] args) throws Exception {
    if (args.length != 1) throw new IllegalArgumentException("Supply an existing test bundle");
    Path bundle = Path.of(args[0]).toAbsolutePath().normalize();
    Path root = Files.createTempDirectory(Path.of("target"), "portable-jar-test-");
    String resource = "nu/pattern/opencv/linux/x86_64/libopencv_java490.so";
    Path upstream = root.resolve("upstream.jar"), nativeFile = root.resolve("replacement.so"), patched = root.resolve("patched.jar");
    Map<String, byte[]> entries = Map.of("org/opencv/core/Core.class", new byte[] {1, 2, 3},
        "META-INF/MANIFEST.MF", "Manifest-Version: 1.0\n".getBytes(),
        "nu/pattern/opencv/windows/x86_64/opencv_java490.dll", new byte[] {4, 5, 6},
        resource, new byte[] {7, 8, 9});
    try (ZipOutputStream zip = new ZipOutputStream(Files.newOutputStream(upstream))) {
      for (var e : entries.entrySet()) {
        zip.putNextEntry(new ZipEntry(e.getKey())); zip.write(e.getValue()); zip.closeEntry();
      }
    }
    // Minimal ELF header suffices for resource replacement; it is not claimed to be loadable.
    byte[] elf = new byte[64];
    elf[0] = 0x7f; elf[1] = 'E'; elf[2] = 'L'; elf[3] = 'F'; elf[4] = 2; elf[5] = 1; elf[18] = 62;
    Files.write(nativeFile, elf);
    byte[] original = Files.readAllBytes(upstream);
    PortableRelease.main(new String[] {"patch-opencv", upstream.toString(), nativeFile.toString(), patched.toString()});
    require(Arrays.equals(original, Files.readAllBytes(upstream)), "Original dependency was modified");
    try (ZipFile zip = new ZipFile(patched.toFile())) {
      for (var e : entries.entrySet()) {
        byte[] actual = zip.getInputStream(zip.getEntry(e.getKey())).readAllBytes();
        require(Arrays.equals(e.getKey().equals(resource) ? elf : e.getValue(), actual), "Unexpected resource change: " + e.getKey());
      }
    }
    fails("Refusing to overwrite", "patch-opencv", upstream.toString(), nativeFile.toString(), patched.toString());
    Files.write(nativeFile, new byte[] {0, 1, 2});
    fails("Linux x86-64 ELF", "patch-opencv", upstream.toString(), nativeFile.toString(), root.resolve("bad.jar").toString());
    require(!Files.exists(root.resolve("bad.jar")), "Invalid binary created an output dependency");
    PortableRelease.main(new String[] {"verify", bundle.toString()});
    checks++;
    Path metadata = bundle.resolve("portable-release.properties");
    byte[] saved = Files.readAllBytes(metadata);
    try {
      Files.writeString(metadata, "tampered=true\n");
      fails("checksum failed", "verify", bundle.toString());
    } finally { Files.write(metadata, saved); }
    Path extra = bundle.resolve("unlisted-packaging-test.txt");
    if (Files.exists(extra)) throw new IOException("Unexpected pre-existing fixture");
    try {
      Files.writeString(extra, "extra");
      fails("missing/unlisted", "verify", bundle.toString());
    } finally { Files.delete(extra); }
    PortableRelease.main(new String[] {"verify", bundle.toString()});
    fails("Refusing to overwrite", "pack", "--root", ".", "--runtime", System.getProperty("java.home"),
        "--platform", "windows-x64", "--version", "test", "--output", bundle.toString());
    fails("Invalid release option", "pack", "--unknown", "value");
    System.out.println("PASS: " + checks + " packaging integrity checks");
  }
}
