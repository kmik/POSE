package sgm;

import java.io.*;
import java.util.zip.GZIPOutputStream;

final class Compression {
  static OutputStream gzip(OutputStream out, int level) throws IOException {
    return new GZIPOutputStream(out, 65536) {
      {
        def.setLevel(level);
      }
    };
  }
}
