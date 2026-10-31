package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.lang.reflect.Method;
import java.util.*;

/**
 * Research only: two complete path sweeps, path-minimum candidates, no aggregate spool. This is an
 * eSGM-style candidate experiment, not the paper's optimized three-pass schedule. Candidate-only
 * runner-up costs MUST NOT be substituted for production uniqueness costs.
 */
final class EsgmPrototype {
  record Result(
      int[] summary,
      short[] outsideBounds,
      short[] candidates,
      short[] sums,
      long deviceBytes,
      double seconds,
      Object profile) {}

  static long deviceBytes(int w, int h, int d, int rows, Options o) {
    long n = (long) w * h, c = o.directions;
    return n * (16L * ((o.census * o.census - 1 + 63) / 64) + 6 + c * 8)
        + 2L * w * rows * d
        + 8L * w * d
        + 24L * w * rows
        + 64;
  }

  static Result run(
      Options o,
      byte[] a,
      byte[] b,
      boolean[] ma,
      boolean[] mb,
      int w,
      int h,
      boolean reverse,
      boolean inspect)
      throws Exception {
    if (!o.matchingCost.equals("census")
        || !o.censusMode.equals("gray")
        || (o.directions != 4 && o.directions != 8)
        || o.disparities < 3
        || o.disparities > 4096
        || o.gpuStripRows < 1
        || StereoCost.aggregateBytes(o) != 2
        || StereoCost.aggregateBound(o) * (o.disparities + 1L) >= Integer.MAX_VALUE)
      throw new IllegalArgumentException(
          "Research prototype requires Census and bounded 16-bit costs");
    long started = System.nanoTime();
    int n = Math.multiplyExact(w, h), d = o.disparities, c = o.directions;
    if (w < o.census + 2
        || h < o.census + 2
        || a.length != n
        || b.length != n
        || ma.length != n
        || mb.length != n)
      throw new IllegalArgumentException("Invalid research stereo raster dimensions");
    int rows = Math.min(h, o.gpuStripRows);
    long bytes = deviceBytes(w, h, d, rows, o);
    GpuSgm.Device device = GpuSgm.selectedDevice(o);
    if (bytes > GpuSgm.budget(device, o))
      throw new IllegalArgumentException("Prototype working set exceeds GPU budget: " + bytes);
    long candidateBytes = 2L * n * c, sumBytes = candidateBytes * 3;
    long bank = Math.min(device.allocation(), o.gpuBankMb * 1048576L) / 8 * 8;
    long costBytes = 2L * w * rows * d;
    if (sumBytes > device.allocation()
        || candidateBytes > device.allocation()
        || costBytes > bank * 8
        || 4L * w * d > device.allocation())
      throw new IllegalArgumentException("Prototype single-allocation/bank limit exceeded");
    if ((2L * d + GpuSgm.selectedGroup(o)) * 4 > device.localMemory())
      throw new IllegalArgumentException("Prototype path state exceeds local memory");
    try (Arena arena = Arena.ofConfined();
        GpuSgm.Compute cl = new GpuSgm.Compute(o, source())) {
      long[][] da = CensusSgm.census(a, w, h, o.census), db = CensusSgm.census(b, w, h, o.census);
      MemorySegment ga = descriptors(cl, arena, da), gb = descriptors(cl, arena, db);
      MemorySegment ta = upload(cl, arena.allocateFrom(JAVA_BYTE, mask(ma))),
          tb = upload(cl, arena.allocateFrom(JAVA_BYTE, mask(mb))),
          grayA = upload(cl, arena.allocateFrom(JAVA_BYTE, a)),
          grayB = upload(cl, arena.allocateFrom(JAVA_BYTE, b));
      MemorySegment candidates = cl.buffer(candidateBytes),
          sums = cl.buffer(sumBytes),
          outside = cl.buffer(2L * n),
          carry0 = cl.buffer(4L * w * d),
          carry1 = cl.buffer(4L * w * d),
          dummy = cl.buffer(8),
          out = cl.buffer(24L * w * rows);
      MemorySegment[] costs = new MemorySegment[8];
      for (int i = 0; i < 8; i++)
        costs[i] = cl.buffer(Math.max(8, Math.min(bank, costBytes - i * bank)));
      MemorySegment costKernel = cl.kernel("costs"),
          pathKernel = cl.kernel("paths"),
          selectKernel = cl.kernel("esgm_select"),
          clear = cl.kernel("esgm_clear");
      cl.run(clear, (long) n * c * 3, sums, Math.toIntExact((long) n * c * 3));
      cl.run(clear, n, outside, n);
      MemorySegment checkpoint = arena.allocate(2);
      int[][] dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
      for (int phase = 0; phase < 2; phase++) {
        for (int direction = 0; direction < c; direction++) {
          long passStart = System.nanoTime();
          int dx = dirs[direction][0], dy = dirs[direction][1], stripes = (h + rows - 1) / rows;
          MemorySegment incoming = carry0, outgoing = carry1;
          for (int ordinal = 0; ordinal < stripes; ordinal++) {
            int stripe = dy < 0 ? stripes - ordinal - 1 : ordinal;
            int y0 = stripe * rows, actual = Math.min(rows, h - y0);
            List<Object> args = new ArrayList<>(List.of(ga, gb, ta, tb, grayA, grayB));
            banks(args, costs, dummy, bank);
            Collections.addAll(
                args,
                w,
                h,
                d,
                o.minDisparity,
                reverse ? 1 : 0,
                o.census / 2,
                da.length,
                y0,
                actual,
                0,
                1000,
                o.adTruncation,
                o.gradientTruncation);
            cl.run(costKernel, (long) w * actual * d, args.toArray());
            args.clear();
            banks(args, costs, dummy, bank);
            Collections.addAll(
                args,
                ta,
                grayA,
                w,
                h,
                d,
                dx,
                dy,
                StereoCost.penalty(o.p1, o),
                StereoCost.p2(o),
                o.sgmEdgeScale,
                y0,
                actual,
                1,
                ordinal > 0 ? 1 : 0,
                incoming,
                outgoing,
                new GpuSgm.Local(4L * (2 * d + cl.group())),
                candidates,
                sums,
                outside,
                c,
                direction,
                phase);
            long lines = dy == 0 ? actual : dx == 0 ? w : w + actual - 1;
            cl.run(pathKernel, lines * cl.group(), args.toArray());
            MemorySegment swap = incoming;
            incoming = outgoing;
            outgoing = swap;
          }
          cl.transfer(candidates, 0, checkpoint, true); // bounded queue; completed pass timing
          System.out.printf(
              Locale.ROOT,
              "eSGM research %s: sweep %d/2, path %d/%d, %.3f s%n",
              reverse ? "right" : "left",
              phase + 1,
              direction + 1,
              c,
              (System.nanoTime() - passStart) / 1e9);
        }
      }
      int[] summary = new int[n * 6];
      MemorySegment host = arena.allocate(24L * w * rows);
      for (int y0 = 0; y0 < h; y0 += rows) {
        int actual = Math.min(rows, h - y0);
        cl.run(
            selectKernel,
            (long) w * actual,
            candidates,
            sums,
            ta,
            tb,
            out,
            w,
            h,
            d,
            o.minDisparity,
            reverse ? 1 : 0,
            o.census / 2,
            c,
            y0,
            actual);
        MemorySegment data = host.asSlice(0, 24L * w * actual);
        cl.transfer(out, 0, data, true);
        MemorySegment.copy(data, 0, MemorySegment.ofArray(summary), 24L * w * y0, data.byteSize());
      }
      short[] hc = null, hs = null;
      if (inspect) {
        hc = new short[Math.toIntExact(candidateBytes / 2)];
        hs = new short[Math.toIntExact(sumBytes / 2)];
        MemorySegment ch = arena.allocate(candidateBytes), sh = arena.allocate(sumBytes);
        cl.transfer(candidates, 0, ch, true);
        cl.transfer(sums, 0, sh, true);
        MemorySegment.copy(ch, 0, MemorySegment.ofArray(hc), 0, candidateBytes);
        MemorySegment.copy(sh, 0, MemorySegment.ofArray(hs), 0, sumBytes);
      }
      short[] bounds = new short[n];
      MemorySegment bh = arena.allocate(2L * n);
      cl.transfer(outside, 0, bh, true);
      MemorySegment.copy(bh, 0, MemorySegment.ofArray(bounds), 0, 2L * n);
      return new Result(
          summary, bounds, hc, hs, bytes, (System.nanoTime() - started) / 1e9, cl.profile());
    }
  }

  static boolean[] texture(byte[] input, int w, int h, int window) throws Exception {
    Method method =
        CensusSgm.class.getDeclaredMethod(
            "textureMask", byte[].class, int.class, int.class, int.class);
    method.setAccessible(true);
    return (boolean[]) method.invoke(null, input, w, h, window);
  }

  static float[] select(int[] summary, int d, Options o) throws Exception {
    float[] out = new float[summary.length / 6];
    Arrays.fill(out, Float.NaN);
    Method method =
        GpuStereo.class.getDeclaredMethod(
            "select",
            MemorySegment.class,
            int.class,
            int.class,
            int.class,
            Options.class,
            float[].class,
            CensusSgm.Stats.class,
            byte[].class,
            byte[].class);
    method.setAccessible(true);
    method.invoke(
        null,
        MemorySegment.ofArray(summary),
        0,
        out.length,
        d,
        o,
        out,
        new CensusSgm.Stats(),
        null,
        null);
    return out;
  }

  /** A sufficient full-range uniqueness test, not a replacement candidate-only ratio. */
  static float[] guarded(Result result, Options o) throws Exception {
    float[] selected = select(result.summary, o.disparities, o);
    for (int p = 0; p < selected.length; p++) {
      int at = p * 6,
          value = result.summary[at + 1],
          before = result.summary[at + 3],
          after = result.summary[at + 4];
      long threshold = Math.max(1, (long) Math.floor(value * o.uniqueness));
      if ((result.outsideBounds[p] & 65535) - (long) value <= threshold
          || (before >= 0 && before <= value)
          || (after >= 0 && after <= value)) selected[p] = Float.NaN;
    }
    return selected;
  }

  private static MemorySegment upload(GpuSgm.Compute cl, MemorySegment host) {
    MemorySegment buffer = cl.buffer(host.byteSize());
    cl.transfer(buffer, 0, host, false);
    return buffer;
  }

  private static MemorySegment descriptors(GpuSgm.Compute cl, Arena arena, long[][] descriptors) {
    long stride = descriptors[0].length * 8L;
    MemorySegment host = arena.allocate(stride * descriptors.length);
    for (int j = 0; j < descriptors.length; j++)
      MemorySegment.copy(MemorySegment.ofArray(descriptors[j]), 0, host, j * stride, stride);
    return upload(cl, host);
  }

  private static byte[] mask(boolean[] a) {
    byte[] out = new byte[a.length];
    for (int i = 0; i < a.length; i++) out[i] = (byte) (a[i] ? 1 : 0);
    return out;
  }

  private static void banks(
      List<Object> args, MemorySegment[] costs, MemorySegment dummy, long bank) {
    Collections.addAll(args, costs);
    for (int i = 0; i < 8; i++) args.add(dummy);
    args.add((int) (bank / 2));
    args.add((int) (bank / 2));
  }

  private static String source() throws Exception {
    Method method = GpuStereo.class.getDeclaredMethod("kernels", boolean.class, int.class);
    method.setAccessible(true);
    String source = (String) method.invoke(null, false, 2);
    source = replace(source, "cw(i,CB,value,CV);sw(i,SB,0,SV);", "cw(i,CB,value,CV);");
    source =
        replace(
            source,
            "__local int *state) {",
            "__local int *state, __global ushort *candidate, __global ushort *compact, __global"
                + " ushort *outside, int C, int direction, int phase) {");
    source = replace(source, "next[k]=v;sw(at,SB,sr(at,SB,SV)+v,SV);", "next[k]=v;");
    source =
        replace(
            source,
            "barrier(CLK_LOCAL_MEM_FENCE);__local int"
                + " *tmp=prev;prev=next;next=tmp;x+=dx;y+=dy;step++;",
            COLLECT);
    return source + SELECT;
  }

  private static String replace(String source, String before, String after) {
    if (source.indexOf(before) < 0 || source.indexOf(before) != source.lastIndexOf(before))
      throw new IllegalStateException("Research kernel hook changed: " + before);
    return source.replace(before, after);
  }

  private static final String COLLECT =
      """
barrier(CLK_LOCAL_MEM_FENCE);
if(phase==0){
  int key=2147483647;
  for(int k=tid;k<D;k+=L)if(cr(((ulong)y*w+x)*D+k,CB,CV)!=512)key=min(key,next[k]*(D+1)+k);
  mins[tid]=key;barrier(CLK_LOCAL_MEM_FENCE);
  for(int stride=L/2;stride>0;stride/=2){if(tid<stride)mins[tid]=min(mins[tid],mins[tid+stride]);barrier(CLK_LOCAL_MEM_FENCE);}
  if(tid==0)candidate[(size_t)globalp*C+direction]=mins[0]==2147483647?65535:mins[0]%(D+1);
}else{
  for(int j=tid;j<C*3;j+=L){int k=(int)candidate[(size_t)globalp*C+j/3]+j%3-1;
    if(k>=0&&k<D){size_t at=((size_t)globalp*C)*3+j;compact[at]+=next[k];}}
  int lower=2147483647;
  for(int k=tid;k<D;k+=L)if(cr(((ulong)y*w+x)*D+k,CB,CV)!=512){
    int covered=0;for(int j=0;j<C;j++)covered|=abs(k-(int)candidate[(size_t)globalp*C+j])<=1;
    if(!covered)lower=min(lower,next[k]);
  }
  mins[tid]=lower;barrier(CLK_LOCAL_MEM_FENCE);
  for(int stride=L/2;stride>0;stride/=2){if(tid<stride)mins[tid]=min(mins[tid],mins[tid+stride]);barrier(CLK_LOCAL_MEM_FENCE);}
  if(tid==0)outside[globalp]=mins[0]==2147483647?65535:min(65535,(int)outside[globalp]+mins[0]);
}
barrier(CLK_LOCAL_MEM_FENCE);__local int *tmp=prev;prev=next;next=tmp;x+=dx;y+=dy;step++;
""";

  private static final String SELECT =
      """
__kernel void esgm_clear(__global ushort *v,int count){size_t i=get_global_id(0);if(i<(size_t)count)v[i]=0;}
int esgm_valid(int p,int k,__global const uchar *a,__global const uchar *b,int w,int h,int D,int md,int reverse,int r){
  if(k<0||k>=D)return 0;int x=p%w,y=p/w;long target=(long)x+(reverse?(long)md+k:-(long)md-k);
  return x>=r&&x<w-r&&y>=r&&y<h-r&&a[p]&&target>=r&&target<w-r&&b[y*w+(int)target];
}
__kernel void esgm_select(__global const ushort *candidate,__global const ushort *compact,
    __global const uchar *a,__global const uchar *b,__global int *out,
    int w,int h,int D,int md,int reverse,int r,int C,int y0,int rows){
  size_t i=get_global_id(0);if(i>=(size_t)w*rows)return;int p=i+y0*w;
  int best=-1,value=2147483647,second=2147483647,before=-1,after=-1,owner=-1;
  for(int j=0;j<C;j++){int k=candidate[(size_t)p*C+j],v=compact[((size_t)p*C+j)*3+1];
    if(k<D&&(v<value||(v==value&&(best<0||k<best)))){best=k;value=v;owner=j;}}
  if(owner>=0){size_t at=((size_t)p*C+owner)*3;
    if(esgm_valid(p,best-1,a,b,w,h,D,md,reverse,r))before=compact[at];
    if(esgm_valid(p,best+1,a,b,w,h,D,md,reverse,r))after=compact[at+2];
    for(int j=0;j<C*3;j++){int k=(int)candidate[(size_t)p*C+j/3]+j%3-1;
      if(abs(k-best)>1&&esgm_valid(p,k,a,b,w,h,D,md,reverse,r))second=min(second,(int)compact[(size_t)p*C*3+j]);}
  }
  size_t at=i*6;out[at]=best;out[at+1]=value;out[at+2]=second;out[at+3]=before;out[at+4]=after;
  out[at+5]=esgm_valid(p,0,a,b,w,h,D,md,reverse,r)|(esgm_valid(p,D-1,a,b,w,h,D,md,reverse,r)<<1);
}
""";
}
