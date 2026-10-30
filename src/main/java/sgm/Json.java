package sgm;

import java.io.*;
import java.lang.reflect.Array;
import java.nio.file.*;
import java.util.*;

/** Small deterministic JSON writer; non-finite numeric values are represented by null. */
final class Json {
  private Json() {}

  static Map<String, Object> object(Object... pairs) {
    Map<String, Object> m = new LinkedHashMap<>();
    if (pairs.length % 2 != 0)
      throw new IllegalArgumentException("JSON object requires key/value pairs");
    Set<String> names = new HashSet<>();
    for (int i = 0; i < pairs.length; i += 2) {
      String key = (String) pairs[i];
      if (!names.add(key.toLowerCase(Locale.ROOT)))
        throw new IllegalArgumentException(
            "JSON field names must be distinct ignoring case for Windows readers: " + key);
      m.put(key, pairs[i + 1]);
    }
    return m;
  }

  static void write(Path path, Object value) throws IOException {
    Files.createDirectories(path.getParent());
    Files.writeString(path, string(value) + "\n");
  }

  static String string(Object value) {
    StringBuilder out = new StringBuilder();
    append(out, value);
    return out.toString();
  }

  private static void append(StringBuilder out, Object value) {
    if (value == null) {
      out.append("null");
      return;
    }
    if (value instanceof Boolean) {
      out.append(value);
      return;
    }
    if (value instanceof Number) {
      double d = ((Number) value).doubleValue();
      out.append(Double.isFinite(d) ? value.toString() : "null");
      return;
    }
    if (value instanceof Map) {
      out.append('{');
      boolean first = true;
      for (Map.Entry<?, ?> entry : ((Map<?, ?>) value).entrySet()) {
        if (!first) out.append(',');
        first = false;
        quote(out, entry.getKey().toString());
        out.append(':');
        append(out, entry.getValue());
      }
      out.append('}');
      return;
    }
    if (value instanceof Iterable) {
      out.append('[');
      boolean first = true;
      for (Object v : (Iterable<?>) value) {
        if (!first) out.append(',');
        first = false;
        append(out, v);
      }
      out.append(']');
      return;
    }
    if (value.getClass().isArray()) {
      out.append('[');
      for (int i = 0; i < Array.getLength(value); i++) {
        if (i > 0) out.append(',');
        append(out, Array.get(value, i));
      }
      out.append(']');
      return;
    }
    quote(out, value.toString());
  }

  private static void quote(StringBuilder out, String value) {
    out.append('"');
    for (int i = 0; i < value.length(); i++) {
      char c = value.charAt(i);
      switch (c) {
        case '"':
          out.append("\\\"");
          break;
        case '\\':
          out.append("\\\\");
          break;
        case '\n':
          out.append("\\n");
          break;
        case '\r':
          out.append("\\r");
          break;
        case '\t':
          out.append("\\t");
          break;
        default:
          if (c < 32) out.append(String.format(Locale.ROOT, "\\u%04x", (int) c));
          else out.append(c);
      }
    }
    out.append('"');
  }
}
