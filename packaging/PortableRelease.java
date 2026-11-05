import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;
import java.util.regex.Pattern;
import java.util.zip.*;

/** Release tooling only; never compiled into POSE's processing classes. Run with Java 25. */
public class PortableRelease {
  private static final String NATIVE = "nu/pattern/opencv/linux/x86_64/libopencv_java490.so";
  private static final List<String> JARS = List.of(
      "opencv-4.9.0-0.jar", "proj4j-1.4.3.jar", "proj4j-epsg-1.4.3.jar",
      "imageio-tiff-3.15.3.jar", "imageio-core-3.15.3.jar", "imageio-metadata-3.15.3.jar",
      "common-lang-3.15.3.jar", "common-io-3.15.3.jar", "common-image-3.15.3.jar",
      "pdfbox-3.0.8.jar", "pdfbox-io-3.0.8.jar", "fontbox-3.0.8.jar", "commons-logging-1.4.0.jar");

  public static void main(String[] args) throws Exception {
    if (args.length == 0 || args[0].equals("--help")) {
      System.out.println("""
          Java 25 release tooling (does not modify src/ or the source dependency JARs).
          pack --root DIR --runtime JDK_DIR --platform linux-x64|windows-x64
               --version VERSION --output NEW_DIRECTORY
               [--native libopencv_java490.so --opencv-source OPENCV_4_9_SOURCE]
          patch-opencv INPUT_JAR NATIVE_SO NEW_OUTPUT_JAR
          check-linux JDK_DIR NATIVE_SO
          verify BUNDLE_DIRECTORY
          seal BUNDLE_DIRECTORY  (maintainer: regenerate checksums after adding validation logs)
          """);
      return;
    }
    switch (args[0]) {
      case "pack" -> pack(options(args));
      case "patch-opencv" -> {
        if (args.length != 4) throw new IllegalArgumentException("patch-opencv INPUT_JAR NATIVE_SO NEW_OUTPUT_JAR");
        patch(Path.of(args[1]), Path.of(args[2]), Path.of(args[3]));
      }
      case "check-linux" -> {
        if (args.length != 3) throw new IllegalArgumentException("check-linux JDK_DIR NATIVE_SO");
        checkLinux(Path.of(args[1]), Path.of(args[2]));
      }
      case "verify" -> {
        if (args.length != 2) throw new IllegalArgumentException("verify BUNDLE_DIRECTORY");
        verify(Path.of(args[1]).toAbsolutePath().normalize());
      }
      case "seal" -> {
        if (args.length != 2) throw new IllegalArgumentException("seal BUNDLE_DIRECTORY");
        Path root = Path.of(args[1]).toAbsolutePath().normalize();
        Map<String, String> files = hashes(root);
        files.remove("SHA256SUMS");
        writeHashes(root.resolve("SHA256SUMS"), files);
        verify(root);
      }
      default -> throw new IllegalArgumentException("Unknown release command: " + args[0]);
    }
  }

  private static Map<String, String> options(String[] args) {
    Map<String, String> out = new HashMap<>();
    Set<String> allowed = Set.of("root", "runtime", "platform", "version", "output", "native", "opencv-source");
    for (int i = 1; i < args.length; i += 2) {
      if (!args[i].startsWith("--") || i + 1 == args.length || !allowed.contains(args[i].substring(2)))
        throw new IllegalArgumentException("Invalid release option: " + args[i]);
      if (out.put(args[i].substring(2), args[i + 1]) != null)
        throw new IllegalArgumentException("Repeated release option: " + args[i]);
    }
    return out;
  }

  private static String required(Map<String, String> o, String key) {
    String value = o.get(key);
    if (value == null || value.isBlank()) throw new IllegalArgumentException("Required: --" + key);
    return value;
  }

  private static Path path(Map<String, String> o, String key) {
    return Path.of(required(o, key)).toAbsolutePath().normalize();
  }

  private static void pack(Map<String, String> o) throws Exception {
    Path root = path(o, "root"), runtime = path(o, "runtime"), out = path(o, "output");
    String platform = required(o, "platform"), version = required(o, "version");
    if (!Set.of("linux-x64", "windows-x64").contains(platform))
      throw new IllegalArgumentException("Only linux-x64 and windows-x64 are supported");
    if (!version.matches("[A-Za-z0-9][A-Za-z0-9._-]*")) throw new IllegalArgumentException("Invalid release version");
    if (Files.exists(out)) throw new IOException("Refusing to overwrite release directory: " + out);
    if (out.startsWith(runtime) || out.startsWith(root.resolve("src"))
        || out.startsWith(root.resolve("target/classes")) || out.startsWith(root.resolve("lib")))
      throw new IOException("Release output must be separate from its inputs");
    Map<String, String> sourceBefore = hashes(root.resolve("src"));
    if (!Files.isRegularFile(root.resolve("target/classes/photogrammetry.class")))
      throw new IOException("Compile and test the current source with build.sh/build.ps1 before packaging");
    for (String jar : JARS)
      if (!Files.isRegularFile(root.resolve("lib").resolve(jar))) throw new IOException("Missing dependency: " + jar);
    Properties javaRelease = new Properties();
    try (Reader r = Files.newBufferedReader(runtime.resolve("release"))) { javaRelease.load(r); }
    String javaVersion = javaRelease.getProperty("JAVA_VERSION", "").replace("\"", "");
    if (!javaVersion.startsWith("25.")) throw new IOException("Bundle a tested Java 25 runtime, found: " + javaVersion);
    String javaArch = javaRelease.getProperty("OS_ARCH", "").replace("\"", "");
    if (!Set.of("x86_64", "amd64").contains(javaArch)) throw new IOException("Runtime must be x86-64: " + javaArch);
    String javaOs = javaRelease.getProperty("OS_NAME", "").replace("\"", "").toLowerCase(Locale.ROOT);
    boolean linux = platform.equals("linux-x64");
    if (linux ? !javaOs.equals("linux") : !javaOs.startsWith("windows"))
      throw new IOException("Runtime OS does not match --platform: " + javaOs);
    Path javaBinary = runtime.resolve(linux ? "bin/java" : "bin/java.exe");
    if (!Files.isRegularFile(javaBinary)) throw new IOException("Runtime is missing " + javaBinary);
    if (!linux) {
      if (!System.getProperty("os.name").startsWith("Windows"))
        throw new IOException("Validate Windows release packaging on Windows");
      command(javaBinary.toString(), "-version");
    }
    Path nativeFile = null, opencvSource = null;
    if (linux) {
      nativeFile = path(o, "native");
      opencvSource = path(o, "opencv-source");
      if (!Files.isRegularFile(opencvSource.resolve("LICENSE"))) throw new IOException("OpenCV source LICENSE is required");
      checkLinux(runtime, nativeFile);
    }
    Files.createDirectories(out);
    copyTree(runtime, out.resolve("runtime")); // includes runtime legal/ and notices, and preserves symlinks
    copyTree(root.resolve("target/classes"), out.resolve("target/classes"));
    Files.createDirectories(out.resolve("lib"));
    for (String jar : JARS) {
      Path source = root.resolve("lib").resolve(jar), destination = out.resolve("lib").resolve(jar);
      if (linux && jar.startsWith("opencv-")) patch(source, nativeFile, destination);
      else Files.copy(source, destination);
    }
    for (String name : List.of("pose.sh", "photogrammetry.sh", "pose.bat", "photogrammetry.bat",
        "README.md", "THIRD_PARTY_NOTICES.md", "PORTABLE_INSTALL.md"))
      Files.copy(root.resolve(name), out.resolve(name));
    Files.createDirectories(out.resolve("scripts"));
    Files.copy(root.resolve("scripts/java-env.sh"), out.resolve("scripts/java-env.sh"));
    Files.createDirectories(out.resolve("docs"));
    try (var files = Files.list(root)) {
      for (Path p : files.filter(p -> p.toString().endsWith(".md")).toList())
        Files.copy(p, out.resolve("docs").resolve(p.getFileName()));
    }
    if (Files.isDirectory(root.resolve("docs"))) copyTree(root.resolve("docs"), out.resolve("docs"));
    if (linux) copyOpenCvNotices(opencvSource, out.resolve("legal/opencv-4.9.0"));
    if (!sourceBefore.equals(hashes(root.resolve("src")))) throw new IOException("Source files changed during packaging");
    writeHashes(out.resolve("processing-source.sha256"), sourceBefore);
    Files.writeString(out.resolve("portable-release.properties"),
        "version=" + version + "\nplatform=" + platform + "\njava=" + javaVersion
        + "\nopencv=4.9.0\nglibc.minimum=" + (linux ? "2.28" : "not-applicable")
        + "\nnative.sha256=" + (linux ? sha(nativeFile) : "upstream-windows") + "\n");
    writeHashes(out.resolve("SHA256SUMS"), hashes(out));
    verify(out);
    System.out.println("Portable bundle ready: " + out);
  }

  private static void patch(Path jar, Path nativeFile, Path out) throws Exception {
    if (Files.exists(out)) throw new IOException("Refusing to overwrite dependency: " + out);
    byte[] elf = Files.readAllBytes(nativeFile);
    if (elf.length < 20 || elf[0] != 0x7f || elf[1] != 'E' || elf[2] != 'L' || elf[3] != 'F'
        || elf[4] != 2 || elf[5] != 1 || elf[18] != 62 || elf[19] != 0)
      throw new IOException("Native library must be a Linux x86-64 ELF: " + nativeFile);
    Files.createDirectories(out.toAbsolutePath().getParent());
    boolean found = false;
    try (ZipFile in = new ZipFile(jar.toFile()); ZipOutputStream zip = new ZipOutputStream(Files.newOutputStream(out))) {
      for (ZipEntry entry : Collections.list(in.entries())) {
        if (entry.getName().matches("META-INF/[^/]+\\.(SF|RSA|DSA|EC)"))
          throw new IOException("Signed JAR cannot be repackaged without invalidating its signature");
        ZipEntry next = new ZipEntry(entry.getName());
        if (entry.getTime() >= 0) next.setTime(entry.getTime());
        zip.putNextEntry(next);
        if (entry.getName().equals(NATIVE)) { zip.write(elf); found = true; }
        else if (!entry.isDirectory()) try (InputStream stream = in.getInputStream(entry)) { stream.transferTo(zip); }
        zip.closeEntry();
      }
    }
    if (!found) throw new IOException("OpenPnP JAR lacks the expected native resource: " + NATIVE);
    // Verify that every non-native entry, including all Java bindings, is byte-identical.
    try (ZipFile a = new ZipFile(jar.toFile()); ZipFile b = new ZipFile(out.toFile())) {
      for (ZipEntry entry : Collections.list(a.entries())) {
        if (entry.isDirectory() || entry.getName().equals(NATIVE)) continue;
        try (InputStream x = a.getInputStream(entry); InputStream y = b.getInputStream(b.getEntry(entry.getName()))) {
          if (!Arrays.equals(x.readAllBytes(), y.readAllBytes())) throw new IOException("Unexpected JAR change: " + entry.getName());
        }
      }
    }
  }

  private static void checkLinux(Path runtime, Path nativeFile) throws Exception {
    if (!System.getProperty("os.name").equals("Linux"))
      throw new IOException("Linux release validation must run on Linux (no root/container needed)");
    Set<Path> checked = new HashSet<>();
    List<Path> binaries = new ArrayList<>();
    binaries.add(nativeFile);
    try (var walk = Files.walk(runtime)) { binaries.addAll(walk.filter(Files::isRegularFile).toList()); }
    for (Path p : binaries) {
      if (!checked.add(p.toRealPath())) continue;
      byte[] magic = new byte[4];
      try (InputStream input = Files.newInputStream(p)) { if (input.read(magic) != 4) continue; }
      if (!Arrays.equals(magic, new byte[] {0x7f, 'E', 'L', 'F'})) continue;
      String info = command("readelf", "--version-info", p.toString());
      var versions = Pattern.compile("GLIBC_(\\d+)\\.(\\d+)(?:\\.(\\d+))?").matcher(info);
      while (versions.find()) {
        int major = Integer.parseInt(versions.group(1)), minor = Integer.parseInt(versions.group(2));
        int patch = versions.group(3) == null ? 0 : Integer.parseInt(versions.group(3));
        if (major > 2 || major == 2 && (minor > 28 || minor == 28 && patch > 0))
          throw new IOException(p + " requires " + versions.group() + "; portable baseline is glibc 2.28");
      }
    }
    String dependencies = command("readelf", "--dynamic", nativeFile.toString());
    var needed = Pattern.compile("\\(NEEDED\\).*\\[(.*?)\\]").matcher(dependencies);
    Set<String> system = Set.of("libc.so.6", "libm.so.6", "libdl.so.2", "libpthread.so.0", "librt.so.1", "ld-linux-x86-64.so.2");
    while (needed.find()) if (!system.contains(needed.group(1)))
      throw new IOException("OpenCV has an unbundled dependency: " + needed.group(1)
          + ". Build static OpenCV/codec/C++ dependencies with build-linux-release.sh.");
    command(runtime.resolve("bin/java").toString(), "-version");
    System.out.println("PASS: OpenCV/runtime ELF symbols fit glibc 2.28; OpenCV has no external codec/C++ dependency");
  }

  private static String command(String... args) throws Exception {
    Process process = new ProcessBuilder(args).redirectErrorStream(true).start();
    String text = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
    if (process.waitFor() != 0) throw new IOException(String.join(" ", args) + " failed:\n" + text);
    return text;
  }

  private static void copyTree(Path from, Path to) throws IOException {
    try (var walk = Files.walk(from)) {
      for (Path p : walk.toList()) {
        Path dest = to.resolve(from.relativize(p).toString());
        if (Files.isSymbolicLink(p)) {
          Path link = Files.readSymbolicLink(p);
          if (link.isAbsolute() || !p.getParent().resolve(link).normalize().startsWith(from.toAbsolutePath().normalize()))
            throw new IOException("Nonportable symlink: " + p);
          Files.createDirectories(dest.getParent());
          Files.copy(p, dest, LinkOption.NOFOLLOW_LINKS);
        } else if (Files.isDirectory(p)) Files.createDirectories(dest);
        else Files.copy(p, dest, StandardCopyOption.COPY_ATTRIBUTES, StandardCopyOption.REPLACE_EXISTING);
      }
    }
  }

  private static void copyOpenCvNotices(Path source, Path out) throws IOException {
    try (var walk = Files.walk(source)) {
      for (Path p : walk.filter(Files::isRegularFile).toList()) {
        String name = p.getFileName().toString().toLowerCase(Locale.ROOT);
        String relative = source.relativize(p).toString().replace('\\', '/');
        if (!(name.contains("license") || name.contains("copying") || name.contains("copyright")
            || name.contains("notice") || name.startsWith("readme") || relative.equals("3rdparty/zlib/zlib.h"))) continue;
        Path dest = out.resolve(source.relativize(p));
        Files.createDirectories(dest.getParent());
        Files.copy(p, dest);
      }
    }
  }

  private static String sha(Path file) throws Exception {
    MessageDigest digest = MessageDigest.getInstance("SHA-256");
    try (InputStream input = Files.newInputStream(file)) {
      byte[] buffer = new byte[65536];
      for (int n; (n = input.read(buffer)) >= 0;) digest.update(buffer, 0, n);
    }
    return HexFormat.of().formatHex(digest.digest());
  }

  private static Map<String, String> hashes(Path root) throws Exception {
    Map<String, String> out = new TreeMap<>();
    try (var walk = Files.walk(root)) {
      for (Path p : walk.filter(Files::isRegularFile).toList())
        out.put(root.relativize(p).toString().replace('\\', '/'), sha(p));
    }
    return out;
  }

  private static void writeHashes(Path file, Map<String, String> hashes) throws IOException {
    StringBuilder lines = new StringBuilder();
    hashes.forEach((path, hash) -> lines.append(hash).append("  ").append(path).append('\n'));
    Files.writeString(file, lines);
  }

  private static void verify(Path root) throws Exception {
    Set<String> expected = new HashSet<>();
    for (String line : Files.readAllLines(root.resolve("SHA256SUMS"))) {
      if (line.length() < 67 || !line.substring(0, 64).matches("[0-9a-f]{64}") || !line.substring(64, 66).equals("  "))
        throw new IOException("Invalid checksum record");
      String name = line.substring(66);
      Path p = root.resolve(name).normalize();
      if (!p.startsWith(root) || !expected.add(name) || !sha(p).equals(line.substring(0, 64)))
        throw new IOException("Release checksum failed: " + name);
    }
    Set<String> actual = new HashSet<>(hashes(root).keySet());
    actual.remove("SHA256SUMS");
    if (!actual.equals(expected)) throw new IOException("Release has missing/unlisted files");
    System.out.println("PASS: " + expected.size() + " release file checksums");
  }
}
