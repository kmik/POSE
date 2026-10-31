package sgm;

import java.io.*;
import java.util.*;
import java.util.zip.*;
import java.util.zip.GZIPOutputStream;

final class Compression {
  static OutputStream gzip(OutputStream out, int level) throws IOException {
    return new GZIPOutputStream(out, 65536) {
      {
        def.setLevel(level);
      }
    };
  }

  /** Reuse bounded native deflaters when routing repeatedly reopens concatenated gzip members. */
  static final class GzipPool implements AutoCloseable {
    private final ArrayDeque<Deflater> free = new ArrayDeque<>();
    private final List<Deflater> all = new ArrayList<>();
    private final Map<Deflater, byte[]> inputs = new IdentityHashMap<>(),
        outputs = new IdentityHashMap<>();

    OutputStream open(OutputStream output) throws IOException {
      Deflater def = free.pollFirst();
      if (def == null) {
        def = new Deflater(1, true);
        all.add(def);
        inputs.put(def, new byte[65536]);
        outputs.put(def, new byte[65536]);
      } else def.reset();
      final Deflater encoder = def;
      CRC32 crc = new CRC32();
      output.write(new byte[] {31, (byte) 139, 8, 0, 0, 0, 0, 0, 0, (byte) 255});
      DeflaterOutputStream compressed =
          new DeflaterOutputStream(output, encoder, 512) {
            {
              buf = outputs.get(encoder);
            }
          };
      OutputStream body =
          new OutputStream() {
            long size;
            boolean closed;

            public void write(int value) throws IOException {
              byte[] b = {(byte) value};
              write(b, 0, 1);
            }

            public void write(byte[] b, int offset, int length) throws IOException {
              compressed.write(b, offset, length);
              crc.update(b, offset, length);
              size += length;
            }

            public void close() throws IOException {
              if (closed) return;
              closed = true;
              try {
                compressed.finish();
                long a = crc.getValue(), n = size;
                byte[] trailer = new byte[8];
                for (int i = 0; i < 4; i++) trailer[i] = (byte) (a >>> (8 * i));
                for (int i = 0; i < 4; i++) trailer[i + 4] = (byte) (n >>> (8 * i));
                output.write(trailer);
              } finally {
                try {
                  output.close();
                } finally {
                  free.addLast(encoder);
                }
              }
            }
          };
      return new BufferedOutputStream(body, 512) {
        {
          buf = inputs.get(encoder);
        }
      };
    }

    public void close() {
      for (Deflater def : all) def.end();
      all.clear();
      free.clear();
      inputs.clear();
      outputs.clear();
    }
  }
}
