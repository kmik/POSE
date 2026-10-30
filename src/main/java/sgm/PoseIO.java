package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

final class PoseIO {
  static final String HEADER = "image,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz";

  static void read(Path path, List<ImageFrame> frames) throws IOException {
    read(path, frames, false);
  }

  static void read(Path path, List<ImageFrame> frames, boolean allowOtherImages)
      throws IOException {
    Map<String, ImageFrame> names = new HashMap<>();
    for (ImageFrame f : frames) names.put(f.path.getFileName().toString(), f);
    List<String> lines = Files.readAllLines(path);
    if (lines.isEmpty() || !lines.get(0).replace("\ufeff", "").trim().equals(HEADER))
      throw new IllegalArgumentException("Pose CSV header must be " + HEADER);
    for (int line = 1; line < lines.size(); line++) {
      if (lines.get(line).trim().isEmpty()) continue;
      List<String> fields = csv(lines.get(line));
      if (fields.size() != 13)
        throw new IllegalArgumentException("Expected 13 CSV columns on line " + (line + 1));
      ImageFrame f = names.get(fields.get(0));
      if (f == null && allowOtherImages) continue;
      if (f == null)
        throw new IllegalArgumentException("Pose references unknown image: " + fields.get(0));
      if (f.camera.registered)
        throw new IllegalArgumentException("Duplicate pose for " + fields.get(0));
      for (int i = 0; i < 9; i++) f.camera.rotation[i] = Double.parseDouble(fields.get(i + 1));
      for (int i = 0; i < 3; i++) f.camera.translation[i] = Double.parseDouble(fields.get(i + 10));
      for (double v : f.camera.rotation)
        if (!Double.isFinite(v)) throw new IllegalArgumentException("Non-finite pose");
      for (double v : f.camera.translation)
        if (!Double.isFinite(v)) throw new IllegalArgumentException("Non-finite pose");
      double[] product = Math3.mul(f.camera.rotation, Math3.transpose(f.camera.rotation)),
          identity = Math3.identity();
      for (int i = 0; i < 9; i++)
        if (Math.abs(product[i] - identity[i]) > 1e-5)
          throw new IllegalArgumentException("Pose rotation must be orthonormal");
      double[] r = f.camera.rotation;
      double det =
          r[0] * (r[4] * r[8] - r[5] * r[7])
              - r[1] * (r[3] * r[8] - r[5] * r[6])
              + r[2] * (r[3] * r[7] - r[4] * r[6]);
      if (det < 0)
        throw new IllegalArgumentException("Pose rotation must be proper (determinant +1)");
      f.camera.registered = true;
      f.camera.fixed = true;
    }
    for (ImageFrame f : frames)
      if (!f.camera.registered)
        throw new IllegalArgumentException("Missing pose for " + f.path.getFileName());
  }

  static void write(Path path, List<ImageFrame> frames) throws IOException {
    try (BufferedWriter w = Files.newBufferedWriter(path)) {
      w.write(HEADER);
      w.newLine();
      for (ImageFrame f : frames) {
        if (!f.camera.registered) continue;
        w.write(quote(f.path.getFileName().toString()));
        for (double v : f.camera.rotation) w.write("," + Double.toString(v));
        for (double v : f.camera.translation) w.write("," + Double.toString(v));
        w.newLine();
      }
    }
  }

  static String quote(String v) {
    return "\"" + v.replace("\"", "\"\"") + "\"";
  }

  static List<String> csv(String line) {
    List<String> fields = new ArrayList<>();
    StringBuilder field = new StringBuilder();
    boolean quoted = false;
    for (int i = 0; i < line.length(); i++) {
      char c = line.charAt(i);
      if (c == '"') {
        if (quoted && i + 1 < line.length() && line.charAt(i + 1) == '"') {
          field.append('"');
          i++;
        } else quoted = !quoted;
      } else if (c == ',' && !quoted) {
        fields.add(field.toString().trim());
        field.setLength(0);
      } else field.append(c);
    }
    if (quoted) throw new IllegalArgumentException("Unterminated CSV quote");
    fields.add(field.toString().trim());
    return fields;
  }
}
