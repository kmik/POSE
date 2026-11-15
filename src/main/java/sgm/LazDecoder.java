package sgm;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * LASzip decompression to raw LAS point records (the bytes an uncompressed LAS file would hold).
 *
 * <p>Ported from laz-rs (https://github.com/laz-rs/laz-rs, Apache License 2.0; itself a port of
 * LASzip by Martin Isenburg, rapidlasso). Supported: chunked point-wise compression (compressor 1
 * and 2) with item version 2 (POINT10, GPSTIME11, RGB12, BYTE: point formats 0-3 and their extra
 * bytes) and layered compression (compressor 3) with item version 3 or 4 (POINT14, RGB14,
 * RGBNIR14, BYTE14: point formats 6-8 and their extra bytes). Wave packets (formats 4, 5, 9, 10)
 * and the old item version 1 are refused with a clear message.
 *
 * <p>Arithmetic: unsigned 32-bit values are kept in longs; every operation matches the reference
 * bit for bit (verified against an independent decoder, see LidarRegression).
 */
final class LazDecoder {
  private LazDecoder() {}

  static final int DM_LENGTH_SHIFT = 15, DM_MAX_COUNT = 1 << DM_LENGTH_SHIFT;
  static final int BM_LENGTH_SHIFT = 13, BM_MAX_COUNT = 1 << BM_LENGTH_SHIFT;
  static final long AC_MAX_LENGTH = 0xFFFFFFFFL, AC_MIN_LENGTH = 0x01000000L;

  /** One LAZ item from the laszip VLR: type, size in bytes, version. */
  record Item(int type, int size, int version) {}

  /** The laszip VLR (user "laszip encoded", record 22204). */
  record Vlr(int compressor, int coder, int chunkSize, List<Item> items) {
    boolean variableChunks() {
      return chunkSize == -1;
    }

    static Vlr parse(byte[] d) throws IOException {
      if (d.length < 34) throw new IOException("LAZ: laszip VLR too short");
      int compressor = u16(d, 0), coder = u16(d, 2);
      int chunk = (int) u32(d, 12);
      int n = u16(d, 32);
      List<Item> items = new ArrayList<>();
      for (int i = 0; i < n; i++) {
        int at = 34 + 6 * i;
        if (at + 6 > d.length) throw new IOException("LAZ: laszip VLR item list truncated");
        items.add(new Item(u16(d, at), u16(d, at + 2), u16(d, at + 4)));
      }
      if (coder != 0) throw new IOException("LAZ: unsupported entropy coder " + coder);
      return new Vlr(compressor, coder, chunk, items);
    }

    /** Checks the item list can be decoded; returns the decompressed record size. */
    int check() throws IOException {
      int size = 0;
      for (Item it : items) {
        boolean ok =
            switch (it.type) {
              case 0, 6, 7, 8 -> it.version == 2;
              case 10, 11, 12, 14 -> it.version == 3 || it.version == 4;
              default -> false;
            };
        if (!ok)
          throw new IOException(
              "LAZ: item "
                  + name(it.type)
                  + " version "
                  + it.version
                  + " is not supported (supported: point formats 0-3 with LASzip item version 2,"
                  + " point formats 6-8 with version 3/4; no wave packets). Convert the file to LAS"
                  + " (e.g. laszip -i FILE.laz -o FILE.las) and use the LAS file.");
        size += it.size;
      }
      boolean layered = items.stream().anyMatch(i -> i.type >= 10);
      if (layered != (compressor == 3))
        throw new IOException("LAZ: compressor " + compressor + " does not match the item versions");
      if (compressor < 1 || compressor > 3)
        throw new IOException("LAZ: unsupported compressor " + compressor);
      return size;
    }
  }

  static String name(int type) {
    return switch (type) {
      case 0 -> "BYTE";
      case 6 -> "POINT10";
      case 7 -> "GPSTIME11";
      case 8 -> "RGB12";
      case 9 -> "WAVEPACKET13";
      case 10 -> "POINT14";
      case 11 -> "RGB14";
      case 12 -> "RGBNIR14";
      case 13 -> "WAVEPACKET14";
      case 14 -> "BYTE14";
      default -> "type " + type;
    };
  }

  static int u16(byte[] b, int at) {
    return (b[at] & 255) | (b[at + 1] & 255) << 8;
  }

  static long u32(byte[] b, int at) {
    return ((b[at] & 255) | (b[at + 1] & 255) << 8 | (b[at + 2] & 255) << 16 | (long) (b[at + 3] & 255) << 24);
  }

  static int i32(byte[] b, int at) {
    return (int) u32(b, at);
  }

  static long i64(byte[] b, int at) {
    return u32(b, at) | u32(b, at + 4) << 32;
  }

  static void put16(byte[] b, int at, int v) {
    b[at] = (byte) v;
    b[at + 1] = (byte) (v >> 8);
  }

  static void put32(byte[] b, int at, int v) {
    b[at] = (byte) v;
    b[at + 1] = (byte) (v >> 8);
    b[at + 2] = (byte) (v >> 16);
    b[at + 3] = (byte) (v >> 24);
  }

  static void put64(byte[] b, int at, long v) {
    put32(b, at, (int) v);
    put32(b, at + 4, (int) (v >>> 32));
  }

  // ---- Entropy coding.

  /** Adaptive multi-symbol model. */
  static final class Model {
    final int symbols;
    final int[] distribution, count, table;
    int total, cycle, untilUpdate, lastSymbol, tableSize, tableShift;

    Model(int symbols) {
      this.symbols = symbols;
      lastSymbol = symbols - 1;
      if (symbols > 16) {
        int bits = 3;
        while (symbols > 1 << (bits + 2)) bits++;
        tableSize = 1 << bits;
        tableShift = DM_LENGTH_SHIFT - bits;
        table = new int[tableSize + 2];
      } else table = null;
      distribution = new int[symbols];
      count = new int[symbols];
      cycle = symbols;
      java.util.Arrays.fill(count, 1);
      update();
      untilUpdate = cycle = (symbols + 6) >> 1;
    }

    void update() {
      total += cycle;
      if (total > DM_MAX_COUNT) {
        total = 0;
        for (int i = 0; i < symbols; i++) {
          count[i] = (count[i] + 1) >> 1;
          total += count[i];
        }
      }
      long scale = 0x80000000L / total;
      long sum = 0;
      if (table == null) {
        for (int k = 0; k < symbols; k++) {
          distribution[k] = (int) ((scale * sum) >>> (31 - DM_LENGTH_SHIFT));
          sum += count[k];
        }
      } else {
        int s = 0;
        for (int k = 0; k < symbols; k++) {
          distribution[k] = (int) ((scale * sum) >>> (31 - DM_LENGTH_SHIFT));
          sum += count[k];
          int w = distribution[k] >>> tableShift;
          while (s < w) table[++s] = k - 1;
        }
        table[0] = 0;
        while (s <= tableSize) table[++s] = symbols - 1;
      }
      cycle = (5 * cycle) >> 2;
      int max = (symbols + 6) << 3;
      if (cycle > max) cycle = max;
      untilUpdate = cycle;
    }
  }

  /** Adaptive binary model. */
  static final class BitModel {
    int bit0Count = 1, bitCount = 2, bit0Prob = 1 << (BM_LENGTH_SHIFT - 1), untilUpdate = 4, cycle = 4;

    void update() {
      bitCount += cycle;
      if (bitCount > BM_MAX_COUNT) {
        bitCount = (bitCount + 1) >> 1;
        bit0Count = (bit0Count + 1) >> 1;
        if (bit0Count == bitCount) bitCount++;
      }
      long scale = 0x80000000L / bitCount;
      bit0Prob = (int) ((bit0Count * scale) >>> (31 - BM_LENGTH_SHIFT));
      cycle = (5 * cycle) >> 2;
      if (cycle > 64) cycle = 64;
      untilUpdate = cycle;
    }
  }

  /** Arithmetic decoder over a byte range. Reading past the range yields zero bytes. */
  static final class Decoder {
    private byte[] in;
    private int pos, end;
    private long value, length;

    void init(byte[] in, int pos, int end) {
      this.in = in;
      this.pos = pos;
      this.end = end;
      value = (long) next() << 24 | next() << 16 | next() << 8 | next();
      length = AC_MAX_LENGTH;
    }

    private int next() {
      return pos < end ? in[pos++] & 255 : 0;
    }

    int bit(BitModel m) {
      long x = m.bit0Prob * (length >>> BM_LENGTH_SHIFT);
      int sym;
      if (value < x) {
        length = x;
        m.bit0Count++;
        sym = 0;
      } else {
        value -= x;
        length -= x;
        sym = 1;
      }
      if (length < AC_MIN_LENGTH) renorm();
      if (--m.untilUpdate == 0) m.update();
      return sym;
    }

    int symbol(Model m) throws IOException {
      int sym, n;
      long x, y = length;
      if (m.table != null) {
        length >>>= DM_LENGTH_SHIFT;
        if (length == 0) throw new IOException("LAZ: corrupt data (zero interval)");
        long dv = value / length;
        int t = (int) (dv >>> m.tableShift);
        sym = m.table[t];
        n = m.table[t + 1] + 1;
        while (n > sym + 1) {
          int k = (sym + n) >>> 1;
          if (m.distribution[k] > dv) n = k;
          else sym = k;
        }
        x = m.distribution[sym] * length;
        if (sym != m.lastSymbol) y = m.distribution[sym + 1] * length;
      } else {
        x = 0;
        sym = 0;
        length >>>= DM_LENGTH_SHIFT;
        n = m.symbols;
        int k = n >>> 1;
        do {
          long z = length * m.distribution[k];
          if (z > value) {
            n = k;
            y = z;
          } else {
            sym = k;
            x = z;
          }
          k = (sym + n) >>> 1;
        } while (k != sym);
      }
      value -= x;
      length = y - x;
      if (length < AC_MIN_LENGTH) renorm();
      m.count[sym]++;
      if (--m.untilUpdate == 0) m.update();
      return sym;
    }

    long bits(int bits) {
      if (bits > 19) {
        long low = shortValue();
        return bits(bits - 16) << 16 | low;
      }
      length >>>= bits;
      long sym = value / length;
      value -= length * sym;
      if (length < AC_MIN_LENGTH) renorm();
      return sym;
    }

    private long shortValue() {
      length >>>= 16;
      long sym = value / length;
      value -= length * sym;
      if (length < AC_MIN_LENGTH) renorm();
      return sym & 0xffff;
    }

    long u32() {
      long low = shortValue();
      long high = shortValue();
      return high << 16 | low;
    }

    private void renorm() {
      do {
        value = (value << 8 | next()) & 0xFFFFFFFFL;
        length = (length << 8) & 0xFFFFFFFFL;
      } while (length < AC_MIN_LENGTH);
    }
  }

  /** Integer corrector decoding (LASzip IntegerCompressor, decompression side). */
  static final class Integers {
    final int contexts, bitsHigh, corrBits, corrRange, corrMin;
    final Model[] bits, corrector;
    final BitModel corrector0 = new BitModel();
    int k;

    Integers(int bitCount, int contexts) {
      this.contexts = contexts;
      bitsHigh = 8;
      if (bitCount != 0 && bitCount < 32) {
        corrBits = bitCount;
        corrRange = 1 << bitCount;
        corrMin = -(corrRange / 2);
      } else {
        corrBits = 32;
        corrRange = 0;
        corrMin = Integer.MIN_VALUE;
      }
      bits = new Model[contexts];
      for (int i = 0; i < contexts; i++) bits[i] = new Model(corrBits + 1);
      corrector = new Model[corrBits];
      for (int i = 1; i <= corrBits; i++) corrector[i - 1] = new Model(i <= bitsHigh ? 1 << i : 1 << bitsHigh);
    }

    int decompress(Decoder d, int pred, int context) throws IOException {
      k = d.symbol(bits[context]);
      int c;
      if (k != 0) {
        if (k < 32) {
          if (k <= bitsHigh) c = d.symbol(corrector[k - 1]);
          else {
            int k1 = k - bitsHigh;
            c = d.symbol(corrector[k - 1]);
            int c1 = (int) d.bits(k1);
            c = c << k1 | c1;
          }
          if (c >= 1 << (k - 1)) c += 1;
          else c -= (1 << k) - 1;
        } else c = corrMin;
      } else c = d.bit(corrector0);
      int real = pred + c;
      // corrRange is 0 for 32-bit values: both corrections are then no-ops, as in the reference.
      if (real < 0) real += corrRange;
      else if (real >= corrRange) real -= corrRange;
      return real;
    }
  }

  /** Streaming median of five (LASzip StreamingMedian5). */
  static final class Median5 {
    final int[] v = new int[5];
    boolean high = true;

    void add(int x) {
      if (high) {
        if (x < v[2]) {
          v[4] = v[3];
          v[3] = v[2];
          if (x < v[0]) {
            v[2] = v[1];
            v[1] = v[0];
            v[0] = x;
          } else if (x < v[1]) {
            v[2] = v[1];
            v[1] = x;
          } else v[2] = x;
        } else {
          if (x < v[3]) {
            v[4] = v[3];
            v[3] = x;
          } else v[4] = x;
          high = false;
        }
      } else {
        if (v[2] < x) {
          v[0] = v[1];
          v[1] = v[2];
          if (v[4] < x) {
            v[2] = v[3];
            v[3] = v[4];
            v[4] = x;
          } else if (v[3] < x) {
            v[2] = v[3];
            v[3] = x;
          } else v[2] = x;
        } else {
          if (v[1] < x) {
            v[0] = v[1];
            v[1] = x;
          } else v[0] = x;
          high = true;
        }
      }
    }

    int get() {
      return v[2];
    }
  }

  static final int[][] NUMBER_RETURN_MAP = {
    {15, 14, 13, 12, 11, 10, 9, 8},
    {14, 0, 1, 3, 6, 10, 10, 9},
    {13, 1, 2, 4, 7, 11, 11, 10},
    {12, 3, 4, 5, 8, 12, 12, 11},
    {11, 6, 7, 8, 9, 13, 13, 12},
    {10, 10, 11, 12, 13, 14, 14, 13},
    {9, 10, 11, 12, 13, 14, 15, 14},
    {8, 9, 10, 11, 12, 13, 14, 15}
  };
  static final int[][] NUMBER_RETURN_LEVEL = {
    {0, 1, 2, 3, 4, 5, 6, 7},
    {1, 0, 1, 2, 3, 4, 5, 6},
    {2, 1, 0, 1, 2, 3, 4, 5},
    {3, 2, 1, 0, 1, 2, 3, 4},
    {4, 3, 2, 1, 0, 1, 2, 3},
    {5, 4, 3, 2, 1, 0, 1, 2},
    {6, 5, 4, 3, 2, 1, 0, 1},
    {7, 6, 5, 4, 3, 2, 1, 0}
  };
  static final int[][] NUMBER_RETURN_MAP_6CTX = {
    {0, 1, 2, 3, 4, 5, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5},
    {1, 0, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3},
    {2, 1, 2, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3},
    {3, 3, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4},
    {4, 3, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4},
    {5, 3, 4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4},
    {3, 3, 4, 4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4},
    {4, 3, 4, 4, 4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4},
    {4, 3, 4, 4, 4, 4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4},
    {5, 3, 4, 4, 4, 4, 4, 4, 4, 5, 4, 4, 4, 4, 4, 4},
    {5, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 4, 4, 4, 4, 4},
    {5, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 4, 4, 4},
    {5, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 4, 4},
    {5, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 4},
    {5, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    {5, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5}
  };
  static final int[][] NUMBER_RETURN_LEVEL_8CT = new int[16][16];

  static {
    for (int n = 0; n < 16; n++) for (int r = 0; r < 16; r++) NUMBER_RETURN_LEVEL_8CT[n][r] = Math.min(7, Math.abs(n - r));
  }

  static Model lazy(Model[] models, int index, int symbols) {
    Model m = models[index];
    if (m == null) models[index] = m = new Model(symbols);
    return m;
  }

  // ---- Point-wise (item version 2) fields.

  interface Field {
    int size();

    void first(byte[] src, int at, byte[] out, int o);

    void next(Decoder d, byte[] out, int o) throws IOException;
  }

  /** POINT10 version 2 (20 bytes). */
  static final class Point10 implements Field {
    int x, y, z, intensity, bits, cls, angle, user, source;
    final int[] lastIntensity = new int[16], lastHeight = new int[8];
    final Median5[] mx = new Median5[16], my = new Median5[16];
    final Model changed = new Model(64);
    final Model[] angleModels = {new Model(256), new Model(256)};
    final Model[] bitByte = new Model[256], clsModels = new Model[256], userModels = new Model[256];
    final Integers icIntensity = new Integers(16, 4), icSource = new Integers(16, 1),
        icDx = new Integers(32, 2), icDy = new Integers(32, 22), icZ = new Integers(32, 20);

    Point10() {
      for (int i = 0; i < 16; i++) {
        mx[i] = new Median5();
        my[i] = new Median5();
      }
    }

    public int size() {
      return 20;
    }

    public void first(byte[] src, int at, byte[] out, int o) {
      System.arraycopy(src, at, out, o, 20);
      x = i32(src, at);
      y = i32(src, at + 4);
      z = i32(src, at + 8);
      intensity = 0;
      bits = src[at + 14] & 255;
      cls = src[at + 15] & 255;
      angle = src[at + 16] & 255;
      user = src[at + 17] & 255;
      source = u16(src, at + 18);
    }

    public void next(Decoder d, byte[] out, int o) throws IOException {
      int c = d.symbol(changed);
      int r, n, m, l;
      if (c != 0) {
        if ((c & 32) != 0) bits = d.symbol(lazy(bitByte, bits, 256));
        r = bits & 7;
        n = (bits >> 3) & 7;
        m = NUMBER_RETURN_MAP[n][r];
        l = NUMBER_RETURN_LEVEL[n][r];
        if ((c & 16) != 0) {
          intensity = icIntensity.decompress(d, lastIntensity[m], Math.min(m, 3)) & 0xffff;
          lastIntensity[m] = intensity;
        } else intensity = lastIntensity[m];
        if ((c & 8) != 0) cls = d.symbol(lazy(clsModels, cls, 256));
        if ((c & 4) != 0) {
          int v = d.symbol(angleModels[(bits >> 6) & 1]);
          angle = (angle + v) & 255;
        }
        if ((c & 2) != 0) user = d.symbol(lazy(userModels, user, 256));
        if ((c & 1) != 0) source = icSource.decompress(d, source, 0) & 0xffff;
      } else {
        r = bits & 7;
        n = (bits >> 3) & 7;
        m = NUMBER_RETURN_MAP[n][r];
        l = NUMBER_RETURN_LEVEL[n][r];
      }
      int diff = icDx.decompress(d, mx[m].get(), n == 1 ? 1 : 0);
      x += diff;
      mx[m].add(diff);
      int kBits = icDx.k;
      diff = icDy.decompress(d, my[m].get(), (n == 1 ? 1 : 0) + (kBits < 20 ? kBits & ~1 : 20));
      y += diff;
      my[m].add(diff);
      kBits = (icDx.k + icDy.k) / 2;
      z = icZ.decompress(d, lastHeight[l], (n == 1 ? 1 : 0) + (kBits < 18 ? kBits & ~1 : 18));
      lastHeight[l] = z;
      put32(out, o, x);
      put32(out, o + 4, y);
      put32(out, o + 8, z);
      put16(out, o + 12, intensity);
      out[o + 14] = (byte) bits;
      out[o + 15] = (byte) cls;
      out[o + 16] = (byte) angle;
      out[o + 17] = (byte) user;
      put16(out, o + 18, source);
    }
  }

  static final int GPS_MULTI = 500, GPS_MULTI_MINUS = -10;

  /** GPSTIME11 version 2 (8 bytes). */
  static final class GpsTime11 implements Field {
    static final int UNCHANGED = GPS_MULTI - GPS_MULTI_MINUS + 1, CODE_FULL = GPS_MULTI - GPS_MULTI_MINUS + 2,
        TOTAL = GPS_MULTI - GPS_MULTI_MINUS + 6;
    final Model multiModel = new Model(TOTAL), zeroDiff = new Model(6);
    final Integers ic = new Integers(32, 9);
    int last, next;
    final long[] times = new long[4];
    final int[] diffs = new int[4], counters = new int[4];

    public int size() {
      return 8;
    }

    public void first(byte[] src, int at, byte[] out, int o) {
      System.arraycopy(src, at, out, o, 8);
      times[0] = i64(src, at);
    }

    public void next(Decoder d, byte[] out, int o) throws IOException {
      for (int i = 0; i <= 3; i++)
        if (!step(d)) {
          put64(out, o, times[last]);
          return;
        }
      throw new IOException("LAZ: too many GPS time sequence switches (corrupt data)");
    }

    /** True when the sequence switched and the time must be read again. */
    private boolean step(Decoder d) throws IOException {
      if (diffs[last] == 0) {
        int multi = d.symbol(zeroDiff);
        if (multi == 1) {
          diffs[last] = ic.decompress(d, 0, 0);
          times[last] += diffs[last];
          counters[last] = 0;
        } else if (multi == 2) {
          full(d);
        } else if (multi > 2) {
          last = (last + multi - 2) & 3;
          return true;
        }
      } else {
        int multi = d.symbol(multiModel);
        if (multi == 1) {
          times[last] += ic.decompress(d, diffs[last], 1);
          counters[last] = 0;
        } else if (multi < UNCHANGED) {
          int diff;
          if (multi == 0) {
            diff = ic.decompress(d, 0, 7);
            extreme(diff);
          } else if (multi < GPS_MULTI) {
            diff = ic.decompress(d, multi * diffs[last], multi < 10 ? 2 : 3);
          } else if (multi == GPS_MULTI) {
            diff = ic.decompress(d, multi * diffs[last], 4);
            extreme(diff);
          } else {
            multi = GPS_MULTI - multi;
            if (multi > GPS_MULTI_MINUS) diff = ic.decompress(d, multi * diffs[last], 5);
            else {
              diff = ic.decompress(d, GPS_MULTI_MINUS * diffs[last], 6);
              extreme(diff);
            }
          }
          times[last] += diff;
        } else if (multi == CODE_FULL) {
          full(d);
        } else if (multi > CODE_FULL) {
          last = (last + multi - CODE_FULL) & 3;
          return true;
        }
      }
      return false;
    }

    private void extreme(int diff) {
      if (++counters[last] > 3) {
        diffs[last] = diff;
        counters[last] = 0;
      }
    }

    private void full(Decoder d) throws IOException {
      next = (next + 1) & 3;
      long high = ic.decompress(d, (int) (times[last] >> 32), 8);
      times[next] = high << 32 | d.u32();
      last = next;
      diffs[last] = 0;
      counters[last] = 0;
    }
  }

  /** RGB12 version 2 models and the shared colour decoding (also used by RGB14). */
  static final class RgbModels {
    final Model used = new Model(128);
    final Model[] bytes = new Model[6];

    RgbModels() {
      for (int i = 0; i < 6; i++) bytes[i] = new Model(256);
    }
  }

  static int clamp8(int v) {
    return v < 0 ? 0 : Math.min(255, v);
  }

  /** Decodes one colour against {@code last} (r, g, b as unsigned shorts). */
  static void rgb(Decoder d, RgbModels m, int[] last, int[] out) throws IOException {
    int sym = d.symbol(m.used);
    int r, g, b, diff;
    if ((sym & 1) != 0) r = (d.symbol(m.bytes[0]) + (last[0] & 255)) & 255;
    else r = last[0] & 0xff;
    if ((sym & 2) != 0) r |= ((d.symbol(m.bytes[1]) + (last[0] >> 8)) & 255) << 8;
    else r |= last[0] & 0xff00;
    if ((sym & 64) != 0) {
      diff = (r & 255) - (last[0] & 255);
      if ((sym & 4) != 0) g = (d.symbol(m.bytes[2]) + clamp8(diff + (last[1] & 255))) & 255;
      else g = last[1] & 0xff;
      if ((sym & 16) != 0) {
        int corr = d.symbol(m.bytes[4]);
        diff = (diff + (g & 255) - (last[1] & 255)) / 2;
        b = (corr + clamp8(diff + (last[2] & 255))) & 255;
      } else b = last[2] & 0xff;
      diff = (r >> 8) - (last[0] >> 8);
      if ((sym & 8) != 0) g |= ((d.symbol(m.bytes[3]) + clamp8(diff + (last[1] >> 8))) & 255) << 8;
      else g |= last[1] & 0xff00;
      if ((sym & 32) != 0) {
        int corr = d.symbol(m.bytes[5]);
        diff = (diff + (g >> 8) - (last[1] >> 8)) / 2;
        b |= ((corr + clamp8(diff + (last[2] >> 8))) & 255) << 8;
      } else b |= last[2] & 0xff00;
    } else {
      g = r;
      b = r;
    }
    out[0] = r;
    out[1] = g;
    out[2] = b;
  }

  /** RGB12 version 2 (6 bytes). */
  static final class Rgb12 implements Field {
    final RgbModels models = new RgbModels();
    final int[] last = new int[3];

    public int size() {
      return 6;
    }

    public void first(byte[] src, int at, byte[] out, int o) {
      System.arraycopy(src, at, out, o, 6);
      for (int k = 0; k < 3; k++) last[k] = u16(src, at + 2 * k);
    }

    public void next(Decoder d, byte[] out, int o) throws IOException {
      rgb(d, models, last, last);
      for (int k = 0; k < 3; k++) put16(out, o + 2 * k, last[k]);
    }
  }

  /** BYTE (extra bytes) version 2. */
  static final class Bytes10 implements Field {
    final byte[] last;
    final Model[] models;

    Bytes10(int count) {
      last = new byte[count];
      models = new Model[count];
      for (int i = 0; i < count; i++) models[i] = new Model(256);
    }

    public int size() {
      return last.length;
    }

    public void first(byte[] src, int at, byte[] out, int o) {
      System.arraycopy(src, at, out, o, last.length);
      System.arraycopy(src, at, last, 0, last.length);
    }

    public void next(Decoder d, byte[] out, int o) throws IOException {
      for (int i = 0; i < last.length; i++) last[i] = (byte) (last[i] + d.symbol(models[i]));
      System.arraycopy(last, 0, out, o, last.length);
    }
  }

  // ---- Layered (item version 3 and 4) fields.

  /** A layered field: first point, layer sizes, layer bytes, then points with a shared context. */
  interface Layered {
    int size();

    /** Reads the raw first point; may set the context (POINT14 sets it to the scanner channel). */
    int first(byte[] src, int at, byte[] out, int o, int context);

    int sizes(byte[] src, int at);

    int layers(byte[] src, int at);

    /** Decodes the next point; returns the (possibly updated) context. */
    int next(byte[] out, int o, int context) throws IOException;
  }

  /** A layer: its decoder is usable only when the layer has bytes. */
  static final class Layer {
    final Decoder decoder = new Decoder();
    int size;
    boolean active;

    int load(byte[] src, int at) {
      active = size > 0;
      if (active) decoder.init(src, at, at + size);
      return at + size;
    }
  }

  static final class Point14Context {
    boolean unused = true;
    int x, y, z, intensity, bits, flags, cls, user, source;
    short angle;
    long gps;
    boolean gpsChange;
    final int[] lastIntensity = new int[8], lastZ = new int[8];
    final Median5[] mx = new Median5[12], my = new Median5[12];
    final Model[] changed = new Model[8];
    final Model channel = new Model(3), returnGpsSame = new Model(13), gpsMulti = new Model(515), gpsNoDiff = new Model(5);
    final Model[] returns = new Model[16], returnNumber = new Model[16], clsModels = new Model[64], flagModels = new Model[64],
        userModels = new Model[64];
    Integers dx, dy, iz, iIntensity, iAngle, iSource, iGps;
    int last, next;
    final long[] times = new long[4];
    final int[] diffs = new int[4], counters = new int[4];

    Point14Context() {
      for (int i = 0; i < 12; i++) {
        mx[i] = new Median5();
        my[i] = new Median5();
      }
      for (int i = 0; i < 8; i++) changed[i] = new Model(128);
    }

    static Point14Context of(Point14Context p) {
      Point14Context c = new Point14Context();
      c.unused = false;
      c.x = p.x;
      c.y = p.y;
      c.z = p.z;
      c.intensity = p.intensity;
      c.bits = p.bits;
      c.flags = p.flags;
      c.cls = p.cls;
      c.user = p.user;
      c.source = p.source;
      c.angle = p.angle;
      c.gps = p.gps;
      java.util.Arrays.fill(c.lastIntensity, p.intensity);
      java.util.Arrays.fill(c.lastZ, p.z);
      c.times[0] = p.gps;
      c.dx = new Integers(32, 2);
      c.dy = new Integers(32, 22);
      c.iz = new Integers(32, 20);
      c.iIntensity = new Integers(16, 4);
      c.iAngle = new Integers(16, 2);
      c.iSource = new Integers(16, 1);
      c.iGps = new Integers(32, 9);
      c.gpsChange = false;
      return c;
    }

    int returnNumber() {
      return bits & 15;
    }

    int returnCount() {
      return (bits >> 4) & 15;
    }
  }

  /** POINT14 version 3/4 (30 bytes): nine layers. */
  static final class Point14 implements Layered {
    final boolean v4;
    final Layer[] layer = new Layer[9]; // xy+returns+channel, z, class, flags, intensity, angle, user, source, gps
    final Point14Context[] ctx = new Point14Context[4];
    int current;
    static final int GPS_CODE_FULL = GPS_MULTI - GPS_MULTI_MINUS + 1;

    Point14(int version) {
      v4 = version == 4;
      for (int i = 0; i < 9; i++) layer[i] = new Layer();
      for (int i = 0; i < 4; i++) ctx[i] = new Point14Context();
    }

    public int size() {
      return 30;
    }

    public int first(byte[] src, int at, byte[] out, int o, int context) {
      System.arraycopy(src, at, out, o, 30);
      Point14Context p = new Point14Context();
      p.x = i32(src, at);
      p.y = i32(src, at + 4);
      p.z = i32(src, at + 8);
      p.intensity = u16(src, at + 12);
      p.bits = src[at + 14] & 255;
      p.flags = src[at + 15] & 255;
      p.cls = src[at + 16] & 255;
      p.user = src[at + 17] & 255;
      p.angle = (short) u16(src, at + 18);
      p.source = u16(src, at + 20);
      p.gps = i64(src, at + 22);
      current = (p.flags >> 4) & 3;
      ctx[current] = Point14Context.of(p);
      return current;
    }

    public int sizes(byte[] src, int at) {
      for (int i = 0; i < 9; i++) layer[i].size = (int) u32(src, at + 4 * i);
      return at + 36;
    }

    public int layers(byte[] src, int at) {
      for (Layer l : layer) at = l.load(src, at);
      return at;
    }

    public int next(byte[] out, int o, int context) throws IOException {
      Decoder xy = layer[0].decoder;
      Point14Context c = ctx[current];
      int lpr = (c.returnNumber() == 1 ? 1 : 0) + (c.returnNumber() >= c.returnCount() ? 2 : 0) + (c.gpsChange ? 4 : 0);
      int changed = xy.symbol(c.changed[lpr]);
      if ((changed & 64) != 0) {
        int diff = xy.symbol(c.channel);
        int channel = (current + diff + 1) % 4;
        if (ctx[channel].unused) ctx[channel] = Point14Context.of(ctx[current]);
        current = channel;
        if (!v4) context = current;
        Point14Context t = ctx[current];
        t.flags = (t.flags & ~0x30) | (channel << 4);
      }
      if (v4) context = current;
      c = ctx[current];
      boolean sourceChanged = (changed & 32) != 0, gpsChanged = (changed & 16) != 0, angleChanged = (changed & 8) != 0;
      int lastN = c.returnCount(), lastR = c.returnNumber();
      int n = (changed & 4) != 0 ? xy.symbol(lazy(c.returns, lastN, 16)) : lastN;
      c.bits = (c.bits & 0x0f) | (n << 4 & 0xf0);
      int r;
      switch (changed & 3) {
        case 0 -> r = lastR;
        case 1 -> r = (lastR + 1) % 16;
        case 2 -> r = (lastR + 15) % 16;
        default -> {
          if (gpsChanged) r = xy.symbol(lazy(c.returnNumber, lastR, 16));
          else r = (lastR + xy.symbol(c.returnGpsSame) + 2) % 16;
        }
      }
      c.bits = (c.bits & 0xf0) | (r & 0x0f);
      int m = NUMBER_RETURN_MAP_6CTX[n][r], l = NUMBER_RETURN_LEVEL_8CT[n][r];
      int cpr = (r == 1 ? 2 : 0) + (r >= n ? 1 : 0);
      int idx = (m << 1) | (gpsChanged ? 1 : 0);
      int diff = c.dx.decompress(xy, c.mx[idx].get(), n == 1 ? 1 : 0);
      c.x += diff;
      c.mx[idx].add(diff);
      int kBits = c.dx.k;
      diff = c.dy.decompress(xy, c.my[idx].get(), (n == 1 ? 1 : 0) + (kBits < 20 ? kBits & ~1 : 20));
      c.y += diff;
      c.my[idx].add(diff);
      if (layer[1].active) {
        kBits = (c.dx.k + c.dy.k) / 2;
        c.z = c.iz.decompress(layer[1].decoder, c.lastZ[l], (n == 1 ? 1 : 0) + (kBits < 18 ? kBits & ~1 : 18));
        c.lastZ[l] = c.z;
      }
      if (layer[2].active) {
        int ccc = ((c.cls & 0x1f) << 1) + (cpr == 3 ? 1 : 0);
        c.cls = layer[2].decoder.symbol(lazy(c.clsModels, ccc, 256));
      }
      if (layer[3].active) {
        int lastFlags = ((c.flags >> 7) & 1) << 5 | ((c.flags >> 6) & 1) << 4 | (c.flags & 0x0f);
        int f = layer[3].decoder.symbol(lazy(c.flagModels, lastFlags, 64));
        int channel = (c.flags >> 4) & 3;
        c.flags = ((f >> 5) & 1) << 7 | ((f >> 4) & 1) << 6 | (channel << 4) | (f & 0x0f);
      }
      if (layer[4].active) {
        int i = cpr << 1 | (gpsChanged ? 1 : 0);
        c.intensity = c.iIntensity.decompress(layer[4].decoder, c.lastIntensity[i], cpr) & 0xffff;
        c.lastIntensity[i] = c.intensity;
      }
      if (layer[5].active && angleChanged) c.angle = (short) c.iAngle.decompress(layer[5].decoder, c.angle, gpsChanged ? 1 : 0);
      if (layer[6].active) c.user = layer[6].decoder.symbol(lazy(c.userModels, c.user / 4, 256));
      if (layer[7].active && sourceChanged) c.source = c.iSource.decompress(layer[7].decoder, c.source, 0) & 0xffff;
      c.gpsChange = gpsChanged;
      if (layer[8].active && gpsChanged) {
        int i = 0;
        for (; i <= 3; i++) if (!gpsStep(c, layer[8].decoder)) break;
        if (i > 3) throw new IOException("LAZ: too many GPS time sequence switches (corrupt data)");
        c.gps = c.times[c.last];
      }
      put32(out, o, c.x);
      put32(out, o + 4, c.y);
      put32(out, o + 8, c.z);
      put16(out, o + 12, c.intensity);
      out[o + 14] = (byte) c.bits;
      out[o + 15] = (byte) c.flags;
      out[o + 16] = (byte) c.cls;
      out[o + 17] = (byte) c.user;
      put16(out, o + 18, c.angle);
      put16(out, o + 20, c.source);
      put64(out, o + 22, c.gps);
      return context;
    }

    private static boolean gpsStep(Point14Context c, Decoder d) throws IOException {
      if (c.diffs[c.last] == 0) {
        int multi = d.symbol(c.gpsNoDiff);
        if (multi == 0) {
          c.diffs[c.last] = c.iGps.decompress(d, 0, 0);
          c.times[c.last] += c.diffs[c.last];
          c.counters[c.last] = 0;
        } else if (multi == 1) {
          full(c, d);
        } else {
          c.last = (c.last + multi - 1) & 3;
          return true;
        }
      } else {
        int multi = d.symbol(c.gpsMulti);
        if (multi == 1) {
          c.times[c.last] += c.iGps.decompress(d, c.diffs[c.last], 1);
          c.counters[c.last] = 0;
        } else if (multi < GPS_CODE_FULL) {
          int diff;
          if (multi == 0) {
            diff = c.iGps.decompress(d, 0, 7);
            extreme(c, diff);
          } else if (multi < GPS_MULTI) {
            diff = c.iGps.decompress(d, multi * c.diffs[c.last], multi < 10 ? 2 : 3);
          } else if (multi == GPS_MULTI) {
            diff = c.iGps.decompress(d, GPS_MULTI * c.diffs[c.last], 4);
            extreme(c, diff);
          } else {
            multi = GPS_MULTI - multi;
            if (multi > GPS_MULTI_MINUS) diff = c.iGps.decompress(d, multi * c.diffs[c.last], 5);
            else {
              diff = c.iGps.decompress(d, GPS_MULTI_MINUS * c.diffs[c.last], 6);
              extreme(c, diff);
            }
          }
          c.times[c.last] += diff;
        } else if (multi == GPS_CODE_FULL) {
          full(c, d);
        } else {
          c.last = (c.last + multi - GPS_CODE_FULL) & 3;
          return true;
        }
      }
      return false;
    }

    private static void extreme(Point14Context c, int diff) {
      if (++c.counters[c.last] > 3) {
        c.diffs[c.last] = diff;
        c.counters[c.last] = 0;
      }
    }

    private static void full(Point14Context c, Decoder d) throws IOException {
      c.next = (c.next + 1) & 3;
      long high = c.iGps.decompress(d, (int) (c.times[c.last] >> 32), 8);
      c.times[c.next] = high << 32 | d.u32();
      c.last = c.next;
      c.diffs[c.last] = 0;
      c.counters[c.last] = 0;
    }
  }

  /** RGB14 (and the RGB part of RGBNIR14) version 3/4. */
  static final class Rgb14 implements Layered {
    final boolean v4;
    final Layer layer = new Layer();
    final RgbModels[] models = {new RgbModels(), new RgbModels(), new RgbModels(), new RgbModels()};
    final boolean[] unused = new boolean[4];
    final int[][] last = new int[4][3];
    int used;

    Rgb14(int version) {
      v4 = version == 4;
    }

    public int size() {
      return 6;
    }

    public int first(byte[] src, int at, byte[] out, int o, int context) {
      System.arraycopy(src, at, out, o, 6);
      java.util.Arrays.fill(unused, true);
      for (int k = 0; k < 3; k++) last[context][k] = u16(src, at + 2 * k);
      unused[context] = false;
      used = context;
      return context;
    }

    public int sizes(byte[] src, int at) {
      layer.size = (int) u32(src, at);
      return at + 4;
    }

    public int layers(byte[] src, int at) {
      return layer.load(src, at);
    }

    public int next(byte[] out, int o, int context) throws IOException {
      int lastIdx = used;
      if (used != context) {
        used = context;
        if (unused[context]) {
          last[context] = last[lastIdx].clone();
          unused[context] = false;
          if (!v4) lastIdx = context;
        }
        if (v4) lastIdx = context;
      }
      if (layer.active) {
        int[] fresh = new int[3];
        rgb(layer.decoder, models[used], last[lastIdx], fresh);
        last[lastIdx] = fresh;
      }
      for (int k = 0; k < 3; k++) put16(out, o + 2 * k, last[lastIdx][k]);
      return context;
    }
  }

  /** The NIR part of RGBNIR14 version 3/4. */
  static final class Nir14 implements Layered {
    final boolean v4;
    final Layer layer = new Layer();
    final Model[] used = new Model[4], lower = new Model[4], upper = new Model[4];
    final boolean[] unused = new boolean[4];
    final int[] last = new int[4];
    int usedContext;

    Nir14(int version) {
      v4 = version == 4;
      for (int i = 0; i < 4; i++) {
        used[i] = new Model(4);
        lower[i] = new Model(256);
        upper[i] = new Model(256);
      }
    }

    public int size() {
      return 2;
    }

    public int first(byte[] src, int at, byte[] out, int o, int context) {
      System.arraycopy(src, at, out, o, 2);
      java.util.Arrays.fill(unused, true);
      last[context] = u16(src, at);
      unused[context] = false;
      usedContext = context;
      return context;
    }

    public int sizes(byte[] src, int at) {
      layer.size = (int) u32(src, at);
      return at + 4;
    }

    public int layers(byte[] src, int at) {
      return layer.load(src, at);
    }

    public int next(byte[] out, int o, int context) throws IOException {
      int lastIdx = usedContext;
      if (usedContext != context) {
        usedContext = context;
        if (unused[context]) {
          last[context] = last[lastIdx];
          unused[context] = false;
          if (!v4) lastIdx = context;
        }
        if (v4) lastIdx = context;
      }
      int value = last[lastIdx];
      if (layer.active) {
        Decoder d = layer.decoder;
        int sym = d.symbol(used[usedContext]);
        int v = (sym & 1) != 0 ? (d.symbol(lower[usedContext]) + (value & 255)) & 255 : value & 0xff;
        if ((sym & 2) != 0) v |= ((d.symbol(upper[usedContext]) + (value >> 8)) & 255) << 8;
        else v |= value & 0xff00;
        last[lastIdx] = v;
      }
      put16(out, o, last[lastIdx]);
      return context;
    }
  }

  /** BYTE14 version 3/4: one layer per extra byte. */
  static final class Bytes14 implements Layered {
    final boolean v4;
    final int count;
    final Layer[] layer;
    final Model[][] models = new Model[4][];
    final boolean[] unused = new boolean[4];
    final byte[][] last;
    int used;

    Bytes14(int count, int version) {
      v4 = version == 4;
      this.count = count;
      layer = new Layer[count];
      for (int i = 0; i < count; i++) layer[i] = new Layer();
      last = new byte[4][count];
    }

    private Model[] fresh() {
      Model[] m = new Model[count];
      for (int i = 0; i < count; i++) m[i] = new Model(256);
      return m;
    }

    public int size() {
      return count;
    }

    public int first(byte[] src, int at, byte[] out, int o, int context) {
      System.arraycopy(src, at, out, o, count);
      java.util.Arrays.fill(unused, true);
      System.arraycopy(src, at, last[context], 0, count);
      used = context;
      models[context] = fresh();
      unused[context] = false;
      return context;
    }

    public int sizes(byte[] src, int at) {
      for (int i = 0; i < count; i++) layer[i].size = (int) u32(src, at + 4 * i);
      return at + 4 * count;
    }

    public int layers(byte[] src, int at) {
      for (Layer l : layer) at = l.load(src, at);
      return at;
    }

    public int next(byte[] out, int o, int context) throws IOException {
      int lastIdx = used;
      if (used != context) {
        used = context;
        if (unused[context]) {
          models[context] = fresh();
          unused[context] = false;
          System.arraycopy(last[lastIdx], 0, last[context], 0, count);
          if (!v4) lastIdx = context;
        }
        if (v4) lastIdx = context;
      }
      byte[] b = last[lastIdx];
      Model[] m = models[context];
      for (int i = 0; i < count; i++) if (layer[i].active) b[i] = (byte) (b[i] + layer[i].decoder.symbol(m[i]));
      System.arraycopy(b, 0, out, o, count);
      return context;
    }
  }

  // ---- Chunks.

  /**
   * Decodes one chunk ({@code points} records) from {@code src[at..end)} into {@code out}
   * (records of {@code recordLength} bytes; the decompressed fields fill the leading bytes).
   */
  static void chunk(Vlr vlr, byte[] src, int at, int end, int points, byte[] out, int recordLength) throws IOException {
    if (points <= 0) return;
    if (vlr.compressor == 3) layered(vlr, src, at, points, out, recordLength);
    else pointwise(vlr, src, at, end, points, out, recordLength);
  }

  private static void pointwise(Vlr vlr, byte[] src, int at, int end, int points, byte[] out, int recordLength)
      throws IOException {
    List<Field> fields = new ArrayList<>();
    for (Item it : vlr.items)
      fields.add(
          switch (it.type) {
            case 6 -> new Point10();
            case 7 -> new GpsTime11();
            case 8 -> new Rgb12();
            case 0 -> new Bytes10(it.size);
            default -> throw new IOException("LAZ: unexpected item " + name(it.type));
          });
    int o = 0;
    for (Field f : fields) {
      if (at + f.size() > end) throw new IOException("LAZ: chunk truncated");
      f.first(src, at, out, o);
      at += f.size();
      o += f.size();
    }
    Decoder d = new Decoder();
    d.init(src, at, end);
    for (int p = 1; p < points; p++) {
      o = p * recordLength;
      for (Field f : fields) {
        f.next(d, out, o);
        o += f.size();
      }
    }
  }

  private static void layered(Vlr vlr, byte[] src, int at, int points, byte[] out, int recordLength) throws IOException {
    List<Layered> fields = new ArrayList<>();
    for (Item it : vlr.items)
      switch (it.type) {
        case 10 -> fields.add(new Point14(it.version));
        case 11 -> fields.add(new Rgb14(it.version));
        case 12 -> {
          fields.add(new Rgb14(it.version));
          fields.add(new Nir14(it.version));
        }
        case 14 -> fields.add(new Bytes14(it.size, it.version));
        default -> throw new IOException("LAZ: unexpected item " + name(it.type));
      }
    int o = 0, context = 0;
    for (Layered f : fields) {
      context = f.first(src, at, out, o, context);
      at += f.size();
      o += f.size();
    }
    at += 4; // point count of the chunk
    for (Layered f : fields) at = f.sizes(src, at);
    for (Layered f : fields) at = f.layers(src, at);
    for (int p = 1; p < points; p++) {
      o = p * recordLength;
      context = 0;
      for (Layered f : fields) {
        context = f.next(out, o, context);
        o += f.size();
      }
    }
  }

  /** The chunk table: {points, bytes} per chunk (points are filled in for fixed-size chunks). */
  static long[][] chunkTable(byte[] table, boolean variable) throws IOException {
    if (table.length < 8) throw new IOException("LAZ: chunk table missing");
    long chunks = u32(table, 4);
    if (chunks > Integer.MAX_VALUE / 4) throw new IOException("LAZ: implausible chunk count " + chunks);
    long[][] out = new long[(int) chunks][2];
    if (chunks == 0) return out;
    Decoder d = new Decoder();
    d.init(table, 8, table.length);
    Integers ic = new Integers(32, 2);
    int previousPoints = 0, previousBytes = 0;
    for (int i = 0; i < chunks; i++) {
      if (variable) {
        previousPoints = ic.decompress(d, previousPoints, 0);
        out[i][0] = previousPoints & 0xFFFFFFFFL;
      }
      previousBytes = ic.decompress(d, previousBytes, 1);
      out[i][1] = previousBytes & 0xFFFFFFFFL;
    }
    return out;
  }
}
