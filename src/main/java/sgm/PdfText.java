package sgm;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.zip.DataFormatException;
import java.util.zip.Inflater;

/**
 * Minimal PDF text extraction with the JDK only, enough for machine-written reports (e.g. a
 * calibration report saved from Word): FlateDecode streams, object streams, simple TrueType/Type1
 * fonts (WinAnsi/Standard/MacRoman) and Type0 Identity-H fonts with ToUnicode maps. Each page's
 * text pieces are placed by their text matrix and font widths, then grouped into lines (top to
 * bottom, left to right). Encrypted PDFs, scanned pages and other filters are not supported.
 */
final class PdfText {
  private final byte[] pdf;
  private final String latin;
  private final Map<Integer, String> dicts = new HashMap<>();
  private final Map<Integer, byte[]> streams = new HashMap<>();
  private final Map<Integer, Font> fonts = new HashMap<>();

  private PdfText(byte[] pdf) {
    this.pdf = pdf;
    this.latin = new String(pdf, StandardCharsets.ISO_8859_1);
  }

  /** Text of every page, one string per page, lines separated by '\n'. */
  static List<String> pages(Path file) throws IOException {
    byte[] bytes = Files.readAllBytes(file);
    if (bytes.length < 5 || !new String(bytes, 0, 5, StandardCharsets.ISO_8859_1).equals("%PDF-"))
      throw new IOException(file + " is not a PDF file");
    PdfText p = new PdfText(bytes);
    p.readObjects();
    if (p.latin.contains("/Encrypt")) throw new IOException(file + " is encrypted; save an unprotected copy first");
    return p.pageTexts();
  }

  // ----- objects -----

  private static final Pattern OBJ = Pattern.compile("(?<![0-9])(\\d+)\\s+(\\d+)\\s+obj\\b");

  private void readObjects() throws IOException {
    Matcher m = OBJ.matcher(latin);
    int from = 0;
    while (m.find(from)) {
      int num = Integer.parseInt(m.group(1)), start = m.end();
      int end = latin.indexOf("endobj", start);
      if (end < 0) break;
      int streamAt = streamKeyword(start, end);
      if (streamAt >= 0) {
        String dict = latin.substring(start, streamAt);
        int data = streamAt + 6;
        if (data < pdf.length && pdf[data] == '\r') data++;
        if (data < pdf.length && pdf[data] == '\n') data++;
        int length = -1;
        Matcher len = Pattern.compile("/Length\\s+(\\d+)(\\s+\\d+\\s+R)?").matcher(dict);
        if (len.find() && len.group(2) == null) length = Integer.parseInt(len.group(1));
        int stop = latin.indexOf("endstream", data);
        if (length < 0 || data + length > pdf.length || (stop >= 0 && data + length > stop)) length = Math.max(0, stop - data);
        // endobj may sit inside binary data before the real one: continue after endstream.
        end = Math.max(end, latin.indexOf("endobj", data + length));
        dicts.put(num, dict);
        streams.put(num, Arrays.copyOfRange(pdf, data, data + length));
      } else dicts.put(num, latin.substring(start, end));
      from = end + 6;
    }
    // Objects packed into object streams (PDF 1.5+).
    for (int num : new ArrayList<>(dicts.keySet())) {
      String d = dicts.get(num);
      if (!d.contains("/ObjStm")) continue;
      byte[] data = decoded(num);
      if (data == null) continue;
      String s = new String(data, StandardCharsets.ISO_8859_1);
      int n = intValue(d, "/N"), first = intValue(d, "/First");
      String[] head = s.substring(0, Math.min(first, s.length())).trim().split("\\s+");
      for (int i = 0; i + 1 < head.length && i / 2 < n; i += 2) {
        int id = Integer.parseInt(head[i]), off = first + Integer.parseInt(head[i + 1]);
        int next = i + 3 < head.length ? first + Integer.parseInt(head[i + 3]) : s.length();
        if (off <= s.length()) dicts.putIfAbsent(id, s.substring(off, Math.min(next, s.length())));
      }
    }
  }

  private int streamKeyword(int start, int end) {
    int depth = 0;
    for (int i = start; i < end - 5; i++) {
      char c = latin.charAt(i);
      if (c == '<' && latin.charAt(i + 1) == '<') { depth++; i++; }
      else if (c == '>' && latin.charAt(i + 1) == '>') { depth--; i++; }
      else if (depth == 0 && latin.startsWith("stream", i)) return i;
    }
    return -1;
  }

  private static int intValue(String dict, String key) {
    Matcher m = Pattern.compile(Pattern.quote(key) + "\\s+(\\d+)").matcher(dict);
    return m.find() ? Integer.parseInt(m.group(1)) : 0;
  }

  /** Decoded stream bytes, or null for unsupported filters. */
  private byte[] decoded(int num) {
    byte[] raw = streams.get(num);
    if (raw == null) return null;
    String d = dicts.get(num);
    Matcher f = Pattern.compile("/Filter\\s*(\\[[^\\]]*\\]|/\\w+)").matcher(d);
    if (!f.find()) return raw;
    String filters = f.group(1).replace("[", " ").replace("]", " ").trim();
    if (filters.equals("/FlateDecode")) return inflate(raw);
    return null;
  }

  private static byte[] inflate(byte[] raw) {
    Inflater inf = new Inflater();
    inf.setInput(raw);
    ByteArrayOutputStream out = new ByteArrayOutputStream(raw.length * 4);
    byte[] buf = new byte[65536];
    try {
      while (!inf.finished()) {
        int n = inf.inflate(buf);
        if (n == 0 && (inf.needsInput() || inf.needsDictionary())) break;
        out.write(buf, 0, n);
      }
    } catch (DataFormatException e) {
      // Keep what decoded: Word streams are well formed, others may carry trailing junk.
    } finally {
      inf.end();
    }
    return out.toByteArray();
  }

  /** Resolves "12 0 R" to the object's text; anything else is returned as is. */
  private String resolve(String value) {
    Matcher r = Pattern.compile("^\\s*(\\d+)\\s+\\d+\\s+R").matcher(value);
    return r.find() ? dicts.getOrDefault(Integer.parseInt(r.group(1)), "") : value;
  }

  private static Integer ref(String value) {
    Matcher r = Pattern.compile("^\\s*(\\d+)\\s+\\d+\\s+R").matcher(value);
    return r.find() ? Integer.parseInt(r.group(1)) : null;
  }

  /** The value of a dictionary entry: a nested dictionary, array, reference or single token. */
  private static String entry(String dict, String key) {
    Matcher m = Pattern.compile(Pattern.quote(key) + "(?![A-Za-z0-9])\\s*").matcher(dict);
    while (m.find()) {
      int i = m.end();
      if (i >= dict.length()) return null;
      char c = dict.charAt(i);
      if (c == '<' && i + 1 < dict.length() && dict.charAt(i + 1) == '<') return balanced(dict, i, "<<", ">>");
      if (c == '[') return balanced(dict, i, "[", "]");
      Matcher r = Pattern.compile("\\G(\\d+\\s+\\d+\\s+R|/[^\\s/<>\\[\\]()]+|[-+0-9.]+|\\((?:\\\\.|[^\\\\)])*\\))").matcher(dict);
      if (r.find(i)) return r.group(1);
      return null;
    }
    return null;
  }

  private static String balanced(String s, int at, String open, String close) {
    int depth = 0;
    for (int i = at; i < s.length(); ) {
      if (s.startsWith(open, i)) { depth++; i += open.length(); }
      else if (s.startsWith(close, i)) {
        depth--;
        i += close.length();
        if (depth == 0) return s.substring(at, i);
      } else i++;
    }
    return s.substring(at);
  }

  // ----- pages -----

  private List<String> pageTexts() {
    String catalog = null;
    for (String d : dicts.values()) if (d.matches("(?s).*/Type\\s*/Catalog\\b.*")) { catalog = d; break; }
    List<Integer> pages = new ArrayList<>();
    if (catalog != null && entry(catalog, "/Pages") != null) collectPages(ref(entry(catalog, "/Pages")), pages, new HashSet<>());
    List<String> out = new ArrayList<>();
    for (int page : pages) out.add(pageText(page));
    return out;
  }

  private void collectPages(Integer node, List<Integer> out, Set<Integer> seen) {
    if (node == null || !seen.add(node)) return;
    String d = dicts.getOrDefault(node, "");
    String kids = entry(d, "/Kids");
    if (kids != null && d.matches("(?s).*/Type\\s*/Pages\\b.*")) {
      Matcher r = Pattern.compile("(\\d+)\\s+\\d+\\s+R").matcher(kids);
      while (r.find()) collectPages(Integer.parseInt(r.group(1)), out, seen);
    } else out.add(node);
  }

  private String pageText(int page) {
    String d = dicts.get(page);
    String resources = inherited(page, "/Resources");
    Map<String, Font> pageFonts = new HashMap<>();
    if (resources != null) {
      String fontDict = entry(resolve(resources), "/Font");
      if (fontDict != null) {
        fontDict = resolve(fontDict);
        Matcher f = Pattern.compile("/([^\\s/<>\\[\\]()]+)\\s+(\\d+)\\s+\\d+\\s+R").matcher(fontDict);
        while (f.find()) pageFonts.put(f.group(1), font(Integer.parseInt(f.group(2))));
      }
    }
    ByteArrayOutputStream content = new ByteArrayOutputStream();
    String contents = entry(d, "/Contents");
    if (contents != null) {
      Matcher r = Pattern.compile("(\\d+)\\s+\\d+\\s+R").matcher(contents.startsWith("[") ? contents : contents);
      while (r.find()) {
        int id = Integer.parseInt(r.group(1));
        String target = dicts.getOrDefault(id, "");
        if (!streams.containsKey(id) && target.trim().startsWith("[")) {
          Matcher inner = Pattern.compile("(\\d+)\\s+\\d+\\s+R").matcher(target);
          while (inner.find()) append(content, decoded(Integer.parseInt(inner.group(1))));
        } else append(content, decoded(id));
      }
    }
    List<Piece> pieces = new ArrayList<>();
    new Interpreter(content.toByteArray(), pageFonts, pieces).run();
    return layout(pieces);
  }

  private static void append(ByteArrayOutputStream out, byte[] data) {
    if (data == null) return;
    out.writeBytes(data);
    out.write('\n');
  }

  private String inherited(int node, String key) {
    Set<Integer> seen = new HashSet<>();
    Integer at = node;
    while (at != null && seen.add(at)) {
      String d = dicts.getOrDefault(at, "");
      String v = entry(d, key);
      if (v != null) return v;
      String parent = entry(d, "/Parent");
      at = parent == null ? null : ref(parent);
    }
    return null;
  }

  // ----- fonts -----

  static final class Font {
    boolean twoByte;
    Map<Integer, String> unicode = new HashMap<>();
    Charset charset = Charset.forName("windows-1252");
    int firstChar;
    double[] widths = new double[0];
    Map<Integer, Double> cidWidths = new HashMap<>();
    double defaultWidth = 500;

    double width(int code) {
      if (twoByte) return cidWidths.getOrDefault(code, defaultWidth);
      int i = code - firstChar;
      return i >= 0 && i < widths.length && widths[i] > 0 ? widths[i] : defaultWidth;
    }

    String text(int code) {
      String u = unicode.get(code);
      if (u != null) return u;
      if (twoByte) return "";
      return new String(new byte[] {(byte) code}, charset);
    }
  }

  private Font font(int id) {
    Font cached = fonts.get(id);
    if (cached != null) return cached;
    Font f = new Font();
    String d = dicts.getOrDefault(id, "");
    String encoding = entry(d, "/Encoding");
    f.twoByte = d.matches("(?s).*/Subtype\\s*/Type0\\b.*");
    if (!f.twoByte) {
      if (encoding != null && encoding.contains("MacRoman")) f.charset = Charset.forName("x-MacRoman");
      String fc = entry(d, "/FirstChar");
      f.firstChar = fc == null ? 0 : (int) Double.parseDouble(fc);
      String w = entry(d, "/Widths");
      if (w != null) f.widths = numbers(resolve(w));
      if (encoding != null && !encoding.startsWith("/")) differences(resolve(encoding), f);
    } else {
      String descendants = entry(d, "/DescendantFonts");
      if (descendants != null) {
        Integer cid = ref(descendants.replace("[", "").trim());
        String cd = cid == null ? resolve(descendants) : dicts.getOrDefault(cid, "");
        String dw = entry(cd, "/DW");
        if (dw != null) f.defaultWidth = Double.parseDouble(dw);
        String w = entry(cd, "/W");
        if (w != null) cidWidths(resolve(w), f.cidWidths);
      }
    }
    String tu = entry(d, "/ToUnicode");
    if (tu != null && ref(tu) != null) {
      byte[] cmap = decoded(ref(tu));
      if (cmap != null) toUnicode(new String(cmap, StandardCharsets.ISO_8859_1), f.unicode);
    }
    fonts.put(id, f);
    return f;
  }

  private static final Map<String, String> GLYPHS =
      Map.ofEntries(
          Map.entry("mu", "µ"), Map.entry("plusminus", "±"), Map.entry("degree", "°"),
          Map.entry("space", " "), Map.entry("bullet", "•"), Map.entry("endash", "–"),
          Map.entry("emdash", "—"), Map.entry("quoteright", "’"), Map.entry("quoteleft", "‘"),
          Map.entry("multiply", "×"), Map.entry("copyright", "©"));

  private static void differences(String enc, Font f) {
    String diff = entry(enc, "/Differences");
    if (diff == null) return;
    Matcher m = Pattern.compile("(\\d+)|/([^\\s/\\[\\]]+)").matcher(diff);
    int code = 0;
    while (m.find()) {
      if (m.group(1) != null) code = Integer.parseInt(m.group(1));
      else {
        String g = m.group(2), u = GLYPHS.get(g);
        if (u == null && g.length() == 1) u = g;
        if (u == null && g.matches("uni[0-9A-Fa-f]{4}")) u = String.valueOf((char) Integer.parseInt(g.substring(3), 16));
        if (u != null) f.unicode.put(code, u);
        code++;
      }
    }
  }

  private static double[] numbers(String s) {
    Matcher m = Pattern.compile("[-+]?\\d*\\.?\\d+").matcher(s);
    List<Double> v = new ArrayList<>();
    while (m.find()) v.add(Double.parseDouble(m.group()));
    return v.stream().mapToDouble(Double::doubleValue).toArray();
  }

  private static void cidWidths(String w, Map<Integer, Double> out) {
    // [c [w1 w2 ...]  c1 c2 w ...]
    Matcher m = Pattern.compile("\\[|\\]|[-+]?\\d*\\.?\\d+").matcher(w.substring(1, Math.max(1, w.length() - 1)));
    List<String> t = new ArrayList<>();
    while (m.find()) t.add(m.group());
    for (int i = 0; i < t.size(); ) {
      int c = (int) Double.parseDouble(t.get(i));
      if (i + 1 < t.size() && t.get(i + 1).equals("[")) {
        int j = i + 2;
        while (j < t.size() && !t.get(j).equals("]")) out.put(c++, Double.parseDouble(t.get(j++)));
        i = j + 1;
      } else if (i + 2 < t.size()) {
        int c2 = (int) Double.parseDouble(t.get(i + 1));
        double v = Double.parseDouble(t.get(i + 2));
        for (int k = c; k <= c2; k++) out.put(k, v);
        i += 3;
      } else break;
    }
  }

  private static void toUnicode(String cmap, Map<Integer, String> out) {
    Matcher block = Pattern.compile("(?s)beginbf(char|range)(.*?)endbf\\1").matcher(cmap);
    while (block.find()) {
      List<String> t = new ArrayList<>();
      Matcher tok = Pattern.compile("<([0-9A-Fa-f]*)>|\\[|\\]").matcher(block.group(2));
      while (tok.find()) t.add(tok.group(1) == null ? tok.group() : tok.group(1));
      if (block.group(1).equals("char"))
        for (int i = 0; i + 1 < t.size(); i += 2) out.put(Integer.parseInt(t.get(i), 16), utf16(t.get(i + 1)));
      else
        for (int i = 0; i + 2 < t.size(); ) {
          int lo = Integer.parseInt(t.get(i), 16), hi = Integer.parseInt(t.get(i + 1), 16);
          if (t.get(i + 2).equals("[")) {
            int j = i + 3;
            for (int c = lo; c <= hi && j < t.size() && !t.get(j).equals("]"); c++) out.put(c, utf16(t.get(j++)));
            while (j < t.size() && !t.get(j).equals("]")) j++;
            i = j + 1;
          } else {
            String base = t.get(i + 2);
            int last = Integer.parseInt(base.substring(Math.max(0, base.length() - 4)), 16);
            String prefix = utf16(base.substring(0, Math.max(0, base.length() - 4)));
            for (int c = lo; c <= hi; c++) out.put(c, prefix + (char) (last + c - lo));
            i += 3;
          }
        }
    }
  }

  private static String utf16(String hex) {
    StringBuilder s = new StringBuilder();
    for (int i = 0; i + 4 <= hex.length(); i += 4) s.append((char) Integer.parseInt(hex.substring(i, i + 4), 16));
    if (hex.length() == 2) s.append((char) Integer.parseInt(hex, 16));
    return s.toString();
  }

  // ----- content streams -----

  record Piece(double x, double y, double end, double size, String text) {}

  private static final class Interpreter {
    final byte[] c;
    final Map<String, Font> fonts;
    final List<Piece> out;
    int i;
    final List<Object> operands = new ArrayList<>();
    double[] ctm = {1, 0, 0, 1, 0, 0};
    final Deque<double[]> stack = new ArrayDeque<>();
    double[] tm = {1, 0, 0, 1, 0, 0}, tlm = {1, 0, 0, 1, 0, 0};
    Font font;
    double size = 10, leading, charSpace, wordSpace, hScale = 1;

    Interpreter(byte[] c, Map<String, Font> fonts, List<Piece> out) {
      this.c = c;
      this.fonts = fonts;
      this.out = out;
    }

    void run() {
      while (true) {
        Object t = token();
        if (t == null) return;
        if (t instanceof Op op) {
          operator(op.name);
          operands.clear();
        } else operands.add(t);
      }
    }

    record Op(String name) {}
    record Str(byte[] bytes) {}
    record Name(String name) {}

    double num(int k) {
      Object o = operands.get(operands.size() - k);
      return o instanceof Double d ? d : 0;
    }

    void operator(String op) {
      int n = operands.size();
      switch (op) {
        case "q" -> stack.push(ctm.clone());
        case "Q" -> ctm = stack.isEmpty() ? new double[] {1, 0, 0, 1, 0, 0} : stack.pop();
        case "cm" -> { if (n >= 6) ctm = mul(new double[] {num(6), num(5), num(4), num(3), num(2), num(1)}, ctm); }
        case "BT" -> { tm = new double[] {1, 0, 0, 1, 0, 0}; tlm = tm.clone(); }
        case "Tf" -> {
          if (n >= 2 && operands.get(n - 2) instanceof Name nm) font = fonts.get(nm.name);
          size = num(1);
        }
        case "TL" -> leading = num(1);
        case "Tc" -> charSpace = num(1);
        case "Tw" -> wordSpace = num(1);
        case "Tz" -> hScale = num(1) / 100;
        case "Td" -> { if (n >= 2) move(num(2), num(1)); }
        case "TD" -> { if (n >= 2) { leading = -num(1); move(num(2), num(1)); } }
        case "Tm" -> { if (n >= 6) { tm = new double[] {num(6), num(5), num(4), num(3), num(2), num(1)}; tlm = tm.clone(); } }
        case "T*" -> move(0, -leading);
        case "Tj" -> { if (n >= 1 && operands.get(n - 1) instanceof Str s) show(s.bytes); }
        case "'" -> { move(0, -leading); if (n >= 1 && operands.get(n - 1) instanceof Str s) show(s.bytes); }
        case "\"" -> {
          if (n >= 3) { wordSpace = num(3); charSpace = num(2); }
          move(0, -leading);
          if (n >= 1 && operands.get(n - 1) instanceof Str s) show(s.bytes);
        }
        case "TJ" -> {
          if (n >= 1 && operands.get(n - 1) instanceof List<?> arr)
            for (Object o : arr) {
              if (o instanceof Str s) show(s.bytes);
              else if (o instanceof Double d) tm[4] -= d / 1000 * size * hScale * tm[0];
            }
        }
        default -> {}
      }
    }

    void move(double tx, double ty) {
      tlm = mul(new double[] {1, 0, 0, 1, tx, ty}, tlm);
      tm = tlm.clone();
    }

    void show(byte[] b) {
      if (font == null) return;
      double[] m = mul(tm, ctm);
      double scale = Math.hypot(m[2], m[3]) * size;
      StringBuilder s = new StringBuilder();
      double x0 = m[4], y0 = m[5];
      int step = font.twoByte ? 2 : 1;
      for (int k = 0; k + step <= b.length; k += step) {
        int code = font.twoByte ? ((b[k] & 0xff) << 8) | (b[k + 1] & 0xff) : b[k] & 0xff;
        s.append(font.text(code));
        double adv = (font.width(code) / 1000 * size + charSpace + (step == 1 && code == 32 ? wordSpace : 0)) * hScale;
        tm[4] += adv * tm[0];
        tm[5] += adv * tm[1];
      }
      double[] e = mul(tm, ctm);
      if (s.length() > 0) out.add(new Piece(x0, y0, e[4], scale, s.toString()));
    }

    static double[] mul(double[] a, double[] b) {
      return new double[] {
        a[0] * b[0] + a[1] * b[2], a[0] * b[1] + a[1] * b[3],
        a[2] * b[0] + a[3] * b[2], a[2] * b[1] + a[3] * b[3],
        a[4] * b[0] + a[5] * b[2] + b[4], a[4] * b[1] + a[5] * b[3] + b[5]
      };
    }

    Object token() {
      while (i < c.length) {
        int ch = c[i] & 0xff;
        if (ch == '%') { while (i < c.length && c[i] != '\n' && c[i] != '\r') i++; continue; }
        if (Character.isWhitespace(ch) || ch == 0) { i++; continue; }
        break;
      }
      if (i >= c.length) return null;
      int ch = c[i] & 0xff;
      if (ch == '(') return literal();
      if (ch == '<') {
        if (i + 1 < c.length && c[i + 1] == '<') { skipDict(); return token(); }
        return hex();
      }
      if (ch == '[') {
        i++;
        List<Object> arr = new ArrayList<>();
        while (true) {
          Object t = token();
          if (t == null || (t instanceof Op op && op.name.equals("]"))) break;
          arr.add(t);
        }
        return arr;
      }
      if (ch == ']') { i++; return new Op("]"); }
      if (ch == '/') {
        int s = ++i;
        while (i < c.length && !delimiter(c[i] & 0xff)) i++;
        return new Name(new String(c, s, i - s, StandardCharsets.ISO_8859_1));
      }
      int s = i;
      while (i < c.length && !delimiter(c[i] & 0xff)) i++;
      if (i == s) { i++; return token(); }
      String w = new String(c, s, i - s, StandardCharsets.ISO_8859_1);
      if (w.matches("[-+]?\\d*\\.?\\d+\\.?")) return Double.parseDouble(w.endsWith(".") ? w + "0" : w);
      if (w.equals("BI")) { skipInlineImage(); return token(); }
      return new Op(w);
    }

    static boolean delimiter(int ch) {
      return Character.isWhitespace(ch) || ch == 0 || "()<>[]{}/%".indexOf(ch) >= 0;
    }

    void skipDict() {
      int depth = 0;
      while (i + 1 < c.length) {
        if (c[i] == '<' && c[i + 1] == '<') { depth++; i += 2; }
        else if (c[i] == '>' && c[i + 1] == '>') { depth--; i += 2; if (depth == 0) return; }
        else i++;
      }
      i = c.length;
    }

    void skipInlineImage() {
      for (; i + 2 < c.length; i++)
        if (Character.isWhitespace(c[i]) && c[i + 1] == 'E' && c[i + 2] == 'I'
            && (i + 3 >= c.length || delimiter(c[i + 3] & 0xff))) { i += 3; return; }
      i = c.length;
    }

    Str literal() {
      ByteArrayOutputStream b = new ByteArrayOutputStream();
      int depth = 0;
      i++;
      while (i < c.length) {
        int ch = c[i++] & 0xff;
        if (ch == '\\' && i < c.length) {
          int e = c[i++] & 0xff;
          switch (e) {
            case 'n' -> b.write('\n');
            case 'r' -> b.write('\r');
            case 't' -> b.write('\t');
            case 'b' -> b.write('\b');
            case 'f' -> b.write('\f');
            case '\r' -> { if (i < c.length && c[i] == '\n') i++; }
            case '\n' -> {}
            default -> {
              if (e >= '0' && e <= '7') {
                int v = e - '0';
                for (int k = 0; k < 2 && i < c.length && c[i] >= '0' && c[i] <= '7'; k++) v = v * 8 + (c[i++] - '0');
                b.write(v & 0xff);
              } else b.write(e);
            }
          }
        } else if (ch == '(') { depth++; b.write(ch); }
        else if (ch == ')') { if (depth-- == 0) break; b.write(ch); }
        else b.write(ch);
      }
      return new Str(b.toByteArray());
    }

    Str hex() {
      i++;
      StringBuilder h = new StringBuilder();
      while (i < c.length && c[i] != '>') {
        char ch = (char) c[i++];
        if (Character.digit(ch, 16) >= 0) h.append(ch);
      }
      i++;
      if (h.length() % 2 == 1) h.append('0');
      byte[] out = new byte[h.length() / 2];
      for (int k = 0; k < out.length; k++) out[k] = (byte) Integer.parseInt(h.substring(2 * k, 2 * k + 2), 16);
      return new Str(out);
    }
  }

  /** Lines from top to bottom; pieces on a line left to right, a space where there is a gap. */
  static String layout(List<Piece> pieces) {
    List<Piece> sorted = new ArrayList<>(pieces);
    sorted.sort(Comparator.comparingDouble((Piece p) -> -p.y).thenComparingDouble(p -> p.x));
    List<List<Piece>> lines = new ArrayList<>();
    for (Piece p : sorted) {
      List<Piece> last = lines.isEmpty() ? null : lines.get(lines.size() - 1);
      double tol = Math.max(1, 0.4 * Math.max(p.size, last == null ? 0 : last.get(0).size));
      if (last != null && Math.abs(last.get(0).y - p.y) <= tol) last.add(p);
      else lines.add(new ArrayList<>(List.of(p)));
    }
    StringBuilder s = new StringBuilder();
    for (List<Piece> line : lines) {
      line.sort(Comparator.comparingDouble(Piece::x));
      StringBuilder l = new StringBuilder();
      double end = Double.NEGATIVE_INFINITY;
      for (Piece p : line) {
        if (l.length() > 0 && p.x - end > 0.15 * p.size && l.charAt(l.length() - 1) != ' ' && !p.text.startsWith(" ")) l.append(' ');
        l.append(p.text);
        end = Math.max(end, p.end);
      }
      String text = l.toString().replace(' ', ' ').replaceAll("[ \\t]+", " ").strip();
      if (!text.isEmpty()) s.append(text).append('\n');
    }
    return s.toString();
  }
}
