package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * Strict small JSON reader for orientation metadata; no scripting engine or executable
 * deserialization.
 */
final class JsonRead {
  private final String s;
  private int p;

  private JsonRead(String s) {
    this.s = s;
  }

  static Object read(Path path) throws IOException {
    return parse(Files.readString(path));
  }

  static Object parse(String text) {
    if (text.startsWith("\ufeff")) text = text.substring(1);
    JsonRead r = new JsonRead(text);
    Object v = r.value(0);
    r.space();
    if (r.p != text.length()) throw r.error();
    return v;
  }

  private IllegalArgumentException error() {
    return new IllegalArgumentException("Invalid JSON at character " + p);
  }

  private void space() {
    while (p < s.length() && Character.isWhitespace(s.charAt(p))) p++;
  }

  private boolean take(char c) {
    space();
    if (p < s.length() && s.charAt(p) == c) {
      p++;
      return true;
    }
    return false;
  }

  private Object value(int depth) {
    if (depth > 128) throw error();
    space();
    if (p >= s.length()) throw error();
    char c = s.charAt(p);
    if (c == '"') return string();
    if (take('{')) {
      Map<String, Object> m = new LinkedHashMap<>();
      if (take('}')) return m;
      do {
        space();
        String key = string();
        if (!take(':') || m.containsKey(key)) throw error();
        m.put(key, value(depth + 1));
      } while (take(','));
      if (!take('}')) throw error();
      return m;
    }
    if (take('[')) {
      List<Object> a = new ArrayList<>();
      if (take(']')) return a;
      do {
        a.add(value(depth + 1));
      } while (take(','));
      if (!take(']')) throw error();
      return a;
    }
    for (String literal : List.of("true", "false", "null"))
      if (s.startsWith(literal, p)) {
        p += literal.length();
        return literal.equals("null") ? null : Boolean.valueOf(literal);
      }
    int start = p;
    while (p < s.length() && "0123456789.eE+-".indexOf(s.charAt(p)) >= 0) p++;
    if (start == p) throw error();
    String token = s.substring(start, p);
    if (!token.matches("-?(0|[1-9][0-9]*)(\\.[0-9]+)?([eE][+-]?[0-9]+)?")) throw error();
    double number = Double.parseDouble(token);
    if (!Double.isFinite(number)) throw error();
    return number;
  }

  private String string() {
    if (p >= s.length() || s.charAt(p++) != '"') throw error();
    StringBuilder b = new StringBuilder();
    while (p < s.length()) {
      char c = s.charAt(p++);
      if (c == '"') return b.toString();
      if (c < 32) throw error();
      if (c != '\\') {
        b.append(c);
        continue;
      }
      if (p >= s.length()) throw error();
      c = s.charAt(p++);
      switch (c) {
        case '"', '\\', '/' -> b.append(c);
        case 'b' -> b.append('\b');
        case 'f' -> b.append('\f');
        case 'n' -> b.append('\n');
        case 'r' -> b.append('\r');
        case 't' -> b.append('\t');
        case 'u' -> {
          if (p + 4 > s.length()) throw error();
          try {
            b.append((char) Integer.parseInt(s.substring(p, p + 4), 16));
          } catch (NumberFormatException e) {
            throw error();
          }
          p += 4;
        }
        default -> throw error();
      }
    }
    throw error();
  }

  @SuppressWarnings("unchecked")
  static Map<String, Object> object(Object o) {
    if (!(o instanceof Map)) throw new IllegalArgumentException("Expected JSON object");
    return (Map<String, Object>) o;
  }

  @SuppressWarnings("unchecked")
  static List<Object> array(Object o) {
    if (!(o instanceof List)) throw new IllegalArgumentException("Expected JSON array");
    return (List<Object>) o;
  }

  static double number(Object o) {
    if (!(o instanceof Number)) throw new IllegalArgumentException("Expected JSON number");
    return ((Number) o).doubleValue();
  }

  static double[] numbers(Object o) {
    List<Object> a = array(o);
    double[] v = new double[a.size()];
    for (int i = 0; i < v.length; i++) v[i] = number(a.get(i));
    return v;
  }
}
