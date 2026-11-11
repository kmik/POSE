package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;

/**
 * A plain per-camera depth map file for diagnostics and comparisons: magic "PDM1", width, height
 * (little-endian ints), the pinhole camera (fx, fy, cx, cy, row-major rotation, translation: 16
 * little-endian doubles), then width*height little-endian floats (camera z; 0 = no depth).
 */
final class DepthMapFile {
  private DepthMapFile() {}

  record Map(int width, int height, Camera camera, float[] z) {}

  static void write(Path path, Camera camera, int width, int height, float[] z) throws IOException {
    Files.createDirectories(path.toAbsolutePath().getParent());
    ByteBuffer b = ByteBuffer.allocate(12 + 128 + 4 * z.length).order(ByteOrder.LITTLE_ENDIAN);
    b.put(new byte[] {'P', 'D', 'M', '1'}).putInt(width).putInt(height);
    b.putDouble(camera.fx).putDouble(camera.fy).putDouble(camera.cx).putDouble(camera.cy);
    for (double v : camera.rotation) b.putDouble(v);
    for (double v : camera.translation) b.putDouble(v);
    b.asFloatBuffer().put(z);
    b.position(b.capacity()).flip();
    try (FileChannel ch =
        FileChannel.open(
            path,
            StandardOpenOption.CREATE,
            StandardOpenOption.TRUNCATE_EXISTING,
            StandardOpenOption.WRITE)) {
      while (b.hasRemaining()) ch.write(b);
    }
  }

  static Map read(Path path) throws IOException {
    ByteBuffer b = ByteBuffer.wrap(Files.readAllBytes(path)).order(ByteOrder.LITTLE_ENDIAN);
    if (b.get() != 'P' || b.get() != 'D' || b.get() != 'M' || b.get() != '1')
      throw new IOException("Not a depth map file: " + path);
    int w = b.getInt(), h = b.getInt();
    Camera camera = new Camera(b.getDouble(), b.getDouble(), b.getDouble(), b.getDouble());
    for (int i = 0; i < 9; i++) camera.rotation[i] = b.getDouble();
    for (int i = 0; i < 3; i++) camera.translation[i] = b.getDouble();
    camera.registered = true;
    float[] z = new float[Math.multiplyExact(w, h)];
    b.asFloatBuffer().get(z);
    return new Map(w, h, camera, z);
  }
}
