package sgm;

/**
 * Shared fixed-point cost units. Costs span 0..(census area-1); penalties use 5x5 reference units.
 */
final class StereoCost {
  static int code(Options o) {
    return switch (o.matchingCost) {
      case "census" -> 0;
      case "ad-census" -> 1;
      case "census-gradient" -> 2;
      case "zncc" -> 3;
      default -> throw new IllegalArgumentException("Unknown matching cost");
    };
  }

  static int penalty(double value, Options o) {
    return (int) Math.round(value * (o.census * o.census - 1) / 24.);
  }

  static int p2(Options o) {
    return Math.max(penalty(o.p1, o) + 1, penalty(o.p2, o));
  }

  /**
   * Classic normalized recurrence guarantees 0 <= L <= max(C) + P2. All current costs are <= 120 or
   * the invalid sentinel 512; adaptation never increases P2. Sum is nonnegative and bounded by
   * directions * (512 + effective P2). Carry/local path arithmetic remains signed int for both
   * storage widths.
   */
  static long aggregateBound(Options o) {
    return o.directions * (512L + p2(o));
  }

  static int aggregateBytes(Options o) {
    boolean fits = aggregateBound(o) <= 65535;
    if (o.sgmAggregate.equals("16") && !fits)
      throw new IllegalArgumentException(
          "16-bit SGM aggregate cannot hold proved bound " + aggregateBound(o));
    return !o.sgmAggregate.equals("32") && fits ? 2 : 4;
  }

  /**
   * Subpixel offset in [-0.5, 0.5] from the winner's aggregated cost and its two neighbours. The
   * default parabola is the original formula; equiangular fits a symmetric V (Shimizu/Okutomi).
   * Both GPU and CPU selection call this, so they agree by construction.
   */
  static double subpixel(Options o, double before, double value, double after) {
    if (o.subpixelFit.equals("equiangular")) {
      double rise = Math.max(before, after) - value;
      return rise > 0 ? Math.max(-.5, Math.min(.5, .5 * (before - after) / rise)) : 0;
    }
    double curvature = before + after - 2.0 * value;
    return curvature > 0 ? Math.max(-.5, Math.min(.5, .5 * (before - after) / curvature)) : 0;
  }

  /** Integer edge-aware jump penalty, identical for CPU/GPU and strip-boundary predecessors. */
  static int pathP2(int p1, int p2, int scale, byte[] gray, int current, int previous) {
    if (scale == 0) return p2;
    int difference = Math.abs((gray[current] & 255) - (gray[previous] & 255));
    return Math.max(p1 + 1, (int) ((long) p2 * scale / (scale + difference)));
  }

  /**
   * Matching cost with the opt-in shiftable windows (--census-shift): the minimum of value() over
   * windows whose centres are shifted by up to S pixels, applied identically to both images. Only
   * shifted centres inside the census border of the h rows held by the arrays and textured in both
   * images take part. S = 0 is exactly value(). Mirrored by the OpenCL ccost() in gpuSource().
   */
  static int cost(
      Options o,
      long[][] a,
      long[][] b,
      byte[] grayA,
      byte[] grayB,
      boolean[] ta,
      boolean[] tb,
      int p,
      int q,
      int width,
      int height) {
    int best = value(o, a, b, grayA, grayB, p, q, width);
    int s = o.censusShift;
    if (s == 0) return best;
    int r = o.census / 2, x = p % width, y = p / width, xq = q % width;
    for (int oy = -s; oy <= s; oy++)
      for (int ox = -s; ox <= s; ox++) {
        if (ox == 0 && oy == 0) continue;
        int yy = y + oy, xa = x + ox, xb = xq + ox;
        if (yy < r || yy >= height - r || xa < r || xa >= width - r || xb < r || xb >= width - r)
          continue;
        int pp = yy * width + xa, qq = yy * width + xb;
        if (!ta[pp] || !tb[qq]) continue;
        best = Math.min(best, value(o, a, b, grayA, grayB, pp, qq, width));
      }
    return best;
  }

  /**
   * Census Hamming distance. With --census-edge-mask T, only comparisons whose neighbour lies within
   * T grey levels of the centre of the reference window (grayA) are counted, rescaled to all bits:
   * round(masked * bits / kept). Windows keeping fewer than a quarter of the bits, or lacking the
   * full census rows in grayA, use the plain distance. Bit order matches census()/colorCensus().
   */
  static int hamming(Options o, long[][] a, long[][] b, byte[] grayA, int p, int q, int width) {
    int t = o.censusEdgeMask, r = o.census / 2, y = p / width;
    if (t == 0 || y < r || (long) (y + r + 1) * width > grayA.length) {
      int hamming = 0;
      for (int word = 0; word < a.length; word++)
        hamming += Long.bitCount(a[word][p] ^ b[word][q]);
      return hamming;
    }
    int bits = o.census * o.census - 1, center = grayA[p] & 255, bit = 0, kept = 0;
    long[] mask = new long[a.length];
    for (int dy = -r; dy <= r; dy++)
      for (int dx = -r; dx <= r; dx++) {
        if (dx == 0 && dy == 0) continue;
        if (Math.abs((grayA[p + dy * width + dx] & 255) - center) <= t) {
          mask[bit >> 6] |= 1L << (bit & 63);
          kept++;
        }
        bit++;
      }
    int hamming = 0;
    if (kept * 4 < bits) {
      for (int word = 0; word < a.length; word++)
        hamming += Long.bitCount(a[word][p] ^ b[word][q]);
      return hamming;
    }
    for (int word = 0; word < a.length; word++)
      hamming += Long.bitCount((a[word][p] ^ b[word][q]) & mask[word]);
    return (hamming * bits + kept / 2) / kept;
  }

  /**
   * OpenCL definitions for the opt-in census edge mask / shiftable windows. Empty when both are
   * off, so the default kernels are byte-identical to before. Kernels use ccost() for census-based
   * costs when CENSUS_EXT is defined; the arithmetic mirrors cost()/hamming()/value() exactly.
   */
  static String gpuSource(Options o) {
    if (o.censusEdgeMask == 0 && o.censusShift == 0) return "";
    return "#define CENSUS_EXT 1\n#define CENSUS_MASK "
        + o.censusEdgeMask
        + "\n#define CENSUS_SHIFT "
        + o.censusShift
        + "\n"
        + """
// Indices P/Q are relative to the given buffers, which hold ROWS rows of W pixels with word
// stride N. Identifiers avoid the patterns that WindowedStereo rewrites in kernel text.
int cham(__global const ulong *A,__global const ulong *B,__global const uchar *GA,size_t N,int P,int Q,int W,int ROWS,int R,int WORDS){
  int ham=0,Y=P/W;
  if(CENSUS_MASK==0||Y<R||Y+R>=ROWS){for(int j=0;j<WORDS;j++)ham+=(int)popcount(A[(size_t)j*N+P]^B[(size_t)j*N+Q]);return ham;}
  int bits=(2*R+1)*(2*R+1)-1,c=GA[P],kept=0;
  for(int dy=-R;dy<=R;dy++)for(int dx=-R;dx<=R;dx++){if(dx==0&&dy==0)continue;if(abs((int)GA[P+dy*W+dx]-c)<=CENSUS_MASK)kept++;}
  if(kept*4<bits){for(int j=0;j<WORDS;j++)ham+=(int)popcount(A[(size_t)j*N+P]^B[(size_t)j*N+Q]);return ham;}
  ulong m=0;int bit=0,word=0;
  for(int dy=-R;dy<=R;dy++)for(int dx=-R;dx<=R;dx++){
    if(dx==0&&dy==0)continue;
    if(abs((int)GA[P+dy*W+dx]-c)<=CENSUS_MASK)m|=((ulong)1)<<(bit&63);
    bit++;
    if((bit&63)==0||bit==bits){ham+=(int)popcount((A[(size_t)word*N+P]^B[(size_t)word*N+Q])&m);m=0;word++;}
  }
  return (ham*bits+kept/2)/kept;
}
int cbase(__global const ulong *A,__global const ulong *B,__global const uchar *GA,__global const uchar *GB,size_t N,int P,int Q,int W,int ROWS,int R,int WORDS,int MODE,int WEIGHT,int ADCAP,int GRADCAP){
  int ham=cham(A,B,GA,N,P,Q,W,ROWS,R,WORDS),bits=(2*R+1)*(2*R+1)-1;
  if(MODE==1||MODE==2){int difference,cap;
    if(MODE==1){difference=abs((int)GA[P]-(int)GB[Q]);cap=ADCAP;}
    else{int ax=(int)GA[P+1]-GA[P-1],ay=(int)GA[P+W]-GA[P-W],bx=(int)GB[Q+1]-GB[Q-1],by=(int)GB[Q+W]-GB[Q-W];difference=abs(ax-bx)+abs(ay-by);cap=GRADCAP;}
    int den=1000*cap;return (WEIGHT*ham*cap+(1000-WEIGHT)*min(difference,cap)*bits+den/2)/den;
  }
  return ham;
}
int ccost(__global const ulong *A,__global const ulong *B,__global const uchar *TA,__global const uchar *TB,__global const uchar *GA,__global const uchar *GB,size_t N,int P,int Q,int W,int ROWS,int R,int WORDS,int MODE,int WEIGHT,int ADCAP,int GRADCAP){
  int best=cbase(A,B,GA,GB,N,P,Q,W,ROWS,R,WORDS,MODE,WEIGHT,ADCAP,GRADCAP);
  if(CENSUS_SHIFT>0){int X=P%W,Y=P/W,XQ=Q%W;
    for(int oy=-CENSUS_SHIFT;oy<=CENSUS_SHIFT;oy++)for(int ox=-CENSUS_SHIFT;ox<=CENSUS_SHIFT;ox++){
      if(ox==0&&oy==0)continue;int yy=Y+oy,xa=X+ox,xb=XQ+ox;
      if(yy<R||yy>=ROWS-R||xa<R||xa>=W-R||xb<R||xb>=W-R)continue;
      int pp=yy*W+xa,qq=yy*W+xb;if(!TA[pp]||!TB[qq])continue;
      best=min(best,cbase(A,B,GA,GB,N,pp,qq,W,ROWS,R,WORDS,MODE,WEIGHT,ADCAP,GRADCAP));
    }
  }
  return best;
}
""";
  }

  static int value(
      Options o, long[][] a, long[][] b, byte[] grayA, byte[] grayB, int p, int q, int width) {
    int hamming = hamming(o, a, b, grayA, p, q, width);
    int mode = code(o), bits = o.census * o.census - 1;
    if (mode == 0) return hamming;
    if (mode == 1 || mode == 2) {
      int difference, cap;
      if (mode == 1) {
        difference = Math.abs((grayA[p] & 255) - (grayB[q] & 255));
        cap = o.adTruncation;
      } else {
        int ax = (grayA[p + 1] & 255) - (grayA[p - 1] & 255),
            ay = (grayA[p + width] & 255) - (grayA[p - width] & 255);
        int bx = (grayB[q + 1] & 255) - (grayB[q - 1] & 255),
            by = (grayB[q + width] & 255) - (grayB[q - width] & 255);
        difference = Math.abs(ax - bx) + Math.abs(ay - by);
        cap = o.gradientTruncation;
      }
      int weight = (int) Math.round(o.censusWeight * 1000), denominator = 1000 * cap;
      return (weight * hamming * cap
              + (1000 - weight) * Math.min(difference, cap) * bits
              + denominator / 2)
          / denominator;
    }
    int radius = o.census / 2, n = o.census * o.census;
    long sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    for (int dy = -radius; dy <= radius; dy++)
      for (int dx = -radius; dx <= radius; dx++) {
        int x = grayA[p + dy * width + dx] & 255, y = grayB[q + dy * width + dx] & 255;
        sa += x;
        sb += y;
        saa += x * x;
        sbb += y * y;
        sab += x * y;
      }
    long va = n * saa - sa * sa, vb = n * sbb - sb * sb;
    if (va <= 0 || vb <= 0) return 512;
    double correlation = (n * sab - sa * sb) / Math.sqrt((double) va * vb);
    return (int) Math.round((1 - Math.max(-1, Math.min(1, correlation))) * .5 * bits);
  }
}
