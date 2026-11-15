package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * Content of the Help menu, without a window: version facts, where POSE is installed, the user
 * guide (GUI.md) and a small Markdown to HTML conversion for showing it in a Swing text pane.
 */
final class GuiAbout {
  private GuiAbout() {}

  /** The installation folder: the parent of target/classes (or of lib/ for jars), else null. */
  static Path installRoot() {
    for (String entry : System.getProperty("java.class.path", "").split(File.pathSeparator)) {
      if (entry.isBlank()) continue;
      Path p = Path.of(entry.endsWith("*") ? entry.substring(0, entry.length() - 1) : entry).toAbsolutePath().normalize();
      if (p.endsWith(Path.of("target", "classes")) && p.getParent() != null) return p.getParent().getParent();
    }
    for (String entry : System.getProperty("java.class.path", "").split(File.pathSeparator)) {
      Path p = Path.of(entry.endsWith("*") ? entry.substring(0, entry.length() - 1) : entry).toAbsolutePath().normalize();
      Path dir = Files.isDirectory(p) ? p : p.getParent();
      if (dir != null && dir.getFileName() != null && dir.getFileName().toString().equals("lib") && dir.getParent() != null) return dir.getParent();
    }
    return null;
  }

  /** The user guide: GUI.md next to the program or in docs/ (portable package), else null. */
  static Path guide(Path root) {
    if (root == null) return null;
    for (Path p : List.of(root.resolve("GUI.md"), root.resolve("docs").resolve("GUI.md"))) if (Files.isRegularFile(p)) return p;
    return null;
  }

  /** Folder with the documentation: docs/ when it has Markdown files, else the install folder. */
  static Path docs(Path root) {
    if (root == null) return null;
    Path docs = root.resolve("docs");
    try (var s = Files.list(docs)) {
      if (s.anyMatch(p -> p.toString().endsWith(".md"))) return docs;
    } catch (IOException e) {
      // no docs folder
    }
    return root;
  }

  /** Release metadata of a portable package (portable-release.properties), or empty. */
  static Map<String, String> release(Path root) {
    Map<String, String> out = new LinkedHashMap<>();
    if (root == null) return out;
    Path file = root.resolve("portable-release.properties");
    if (!Files.isRegularFile(file)) return out;
    Properties p = new Properties();
    try (Reader r = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
      p.load(r);
    } catch (IOException e) {
      return out;
    }
    for (String k : new TreeSet<>(p.stringPropertyNames())) out.put(k, p.getProperty(k));
    return out;
  }

  /** Label / value rows of the About box. {@code engine} may be null (computed separately). */
  static List<String[]> facts(Path root, String engine) {
    List<String[]> out = new ArrayList<>();
    out.add(new String[] {"Software", PdfReport.NAME + " " + PdfReport.VERSION + " (" + PdfReport.LONG_NAME + ")"});
    Map<String, String> release = release(root);
    if (release.containsKey("version")) out.add(new String[] {"Release", release.get("version") + (release.containsKey("platform") ? " (" + release.get("platform") + ")" : "")});
    else out.add(new String[] {"Release", "development build (not a portable package)"});
    if (engine != null) out.add(new String[] {"Processing engine", engine.length() > 16 ? engine.substring(0, 16) : engine});
    out.add(new String[] {"Java", System.getProperty("java.version") + " (" + System.getProperty("java.vendor", "") + ")"});
    out.add(new String[] {"Operating system", System.getProperty("os.name") + " " + System.getProperty("os.version") + " " + System.getProperty("os.arch")});
    out.add(new String[] {"Processors", Runtime.getRuntime().availableProcessors() + " (half are used by default)"});
    out.add(new String[] {"Window heap limit", (Runtime.getRuntime().maxMemory() >> 20) + " MiB (processing runs in its own process)"});
    out.add(new String[] {"Installed in", root == null ? "unknown" : root.toString()});
    out.add(new String[] {"Settings", GuiOptions.Settings.user().dir.toString()});
    return out;
  }

  /** The facts as plain text (copied for bug reports). */
  static String text(List<String[]> facts) {
    StringBuilder s = new StringBuilder();
    for (String[] f : facts) s.append(f[0]).append(": ").append(f[1]).append('\n');
    return s.toString();
  }

  /**
   * Markdown subset of the guides as HTML for a Swing text pane: headings, paragraphs, nested
   * "-" / "1." lists, fenced code blocks, inline code, bold and italics.
   */
  static String html(String markdown) {
    StringBuilder h = new StringBuilder("<html><body style='margin:12px'>");
    List<String> paragraph = new ArrayList<>();
    Deque<int[]> lists = new ArrayDeque<>(); // {indent, ordered}
    boolean code = false, item = false;
    for (String raw : (markdown + "\n").split("\r?\n", -1)) {
      if (raw.strip().startsWith("```")) {
        flush(h, paragraph);
        if (!code) {
          item = closeLists(h, lists, -1, item);
          h.append("<pre>");
        } else h.append("</pre>");
        code = !code;
        continue;
      }
      if (code) {
        h.append(escape(raw)).append('\n');
        continue;
      }
      String line = raw.stripTrailing();
      int indent = line.length() - line.stripLeading().length();
      String t = line.strip();
      java.util.regex.Matcher bullet = java.util.regex.Pattern.compile("^([-*]|\\d+\\.)\\s+(.*)$").matcher(t);
      if (t.isEmpty()) {
        flush(h, paragraph);
        continue;
      }
      if (t.startsWith("#")) {
        flush(h, paragraph);
        item = closeLists(h, lists, -1, item);
        int level = (int) Math.min(3, t.chars().takeWhile(ch -> ch == '#').count());
        h.append("<h").append(level).append('>').append(inline(t.replaceFirst("^#+\\s*", ""))).append("</h").append(level).append('>');
        continue;
      }
      if (bullet.find()) {
        flush(h, paragraph);
        boolean ordered = !bullet.group(1).equals("-") && !bullet.group(1).equals("*");
        while (!lists.isEmpty() && lists.peek()[0] > indent) item = closeOne(h, lists, item);
        // A different kind of list at the same level starts a new list.
        if (!lists.isEmpty() && lists.peek()[0] == indent && (lists.peek()[1] == 1) != ordered) {
          closeOne(h, lists, item);
          item = false;
        }
        if (lists.isEmpty() || lists.peek()[0] < indent) {
          h.append(ordered ? "<ol>" : "<ul>");
          lists.push(new int[] {indent, ordered ? 1 : 0});
        } else if (item) h.append("</li>");
        h.append("<li>");
        item = true;
        paragraph.add(bullet.group(2));
        flushInline(h, paragraph);
        continue;
      }
      if (!lists.isEmpty() && indent > lists.peek()[0]) {
        // Continuation of a list item.
        h.append(' ').append(inline(t));
        continue;
      }
      item = closeLists(h, lists, -1, item);
      paragraph.add(t);
    }
    flush(h, paragraph);
    closeLists(h, lists, -1, item);
    if (code) h.append("</pre>");
    return h.append("</body></html>").toString();
  }

  private static boolean closeOne(StringBuilder h, Deque<int[]> lists, boolean item) {
    if (item) h.append("</li>");
    h.append(lists.pop()[1] == 1 ? "</ol>" : "</ul>");
    return !lists.isEmpty();
  }

  private static boolean closeLists(StringBuilder h, Deque<int[]> lists, int indent, boolean item) {
    while (!lists.isEmpty() && lists.peek()[0] > indent) item = closeOne(h, lists, item);
    return lists.isEmpty() ? false : item;
  }

  private static void flush(StringBuilder h, List<String> paragraph) {
    if (paragraph.isEmpty()) return;
    h.append("<p>").append(inline(String.join(" ", paragraph))).append("</p>");
    paragraph.clear();
  }

  private static void flushInline(StringBuilder h, List<String> paragraph) {
    h.append(inline(String.join(" ", paragraph)));
    paragraph.clear();
  }

  /** Inline code, bold and italics; everything else escaped. */
  static String inline(String s) {
    StringBuilder out = new StringBuilder();
    String[] parts = s.split("`", -1);
    for (int i = 0; i < parts.length; i++) {
      if (i % 2 == 1 && i < parts.length - 1) out.append("<code>").append(escape(parts[i])).append("</code>");
      else {
        String e = escape(i % 2 == 1 ? "`" + parts[i] : parts[i]);
        e = e.replaceAll("\\*\\*(.+?)\\*\\*", "<b>$1</b>").replaceAll("(?<![\\w*])\\*(?!\\s)(.+?)(?<!\\s)\\*(?![\\w*])", "<i>$1</i>");
        out.append(e);
      }
    }
    return out.toString();
  }

  static String escape(String s) {
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
  }
}
