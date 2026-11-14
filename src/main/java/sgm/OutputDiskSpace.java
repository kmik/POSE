package sgm;

import java.io.*;
import java.nio.file.*;

/** Actual-space checks for streamed maps/staging, retaining room to export staged cloud points. */
final class OutputDiskSpace {
  interface Available { long bytes() throws IOException; }
  static final int BATCH = 32768;
  private final Available available;
  private final long reserve;
  private long exports, points;

  OutputDiskSpace(Path output, long reserve) {
    this(() -> available(output), reserve);
  }

  OutputDiskSpace(Available available, long reserve) {
    this.available = available;
    this.reserve = reserve;
  }

  static long available(Path output) throws IOException {
    long bytes = Files.getFileStore(output).getUsableSpace();
    Long override = Long.getLong("sgm.mvsgm.diskAvailableMb");
    return override == null ? bytes : Math.min(bytes, Math.multiplyExact(Math.max(0, override), 1L << 20));
  }

  synchronized void exports(long bytes, long pointCount) {
    exports = bytes;
    points = pointCount;
  }

  synchronized void require(long additional, String operation) throws IOException {
    long need = Math.addExact(Math.addExact(additional, exports), reserve);
    long free = available.bytes();
    if (need > free)
      throw new IOException("Insufficient disk for " + operation + ": next allocation "
          + Math.ceilDiv(additional, 1L << 20) + " MiB, pending final cloud exports "
          + Math.ceilDiv(exports, 1L << 20) + " MiB for " + points
          + " staged/batched points, safety reserve " + Math.ceilDiv(reserve, 1L << 20)
          + " MiB; available " + (free >> 20) + " MiB. Reconstruction quality was not changed.");
  }

  static void require(Options o, long bytes, String operation) throws IOException {
    if (o.outputDiskSpace != null) o.outputDiskSpace.require(bytes, operation);
  }
}
