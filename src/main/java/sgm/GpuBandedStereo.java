package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.*;

/**
 * Resident packed-band SGM on OpenCL. Cost arithmetic is copied from GpuStereo; the path transition
 * over shifted predecessor bands follows MvsKernels.paths, inside GpuStereo's continuous line
 * traversal (including diagonals). Results match {@link BandedSgm#cpuSummary} exactly.
 *
 * <p>A session uploads both images' descriptors, texture masks and gray rasters once, then serves
 * left, right and band-edge retry solves; packed volumes grow only when a solve needs more cells.
 */
final class GpuBandedStereo {
  private GpuBandedStereo() {}

  static long deviceBytes(Options o, int w, int h, long cells) {
    long n = (long) w * h, words = (o.census * o.census - 1 + 63) / 64;
    if (o.censusMode.equals("color-distance")) words *= 3;
    return cells * (2L + StereoCost.aggregateBytes(o)) + n * (16L * words + 4 + 12 + 24) + 64;
  }

  static boolean use(Options o, int w, int h, DisparityBands bands) {
    if (o.compute.equals("cpu")) return false;
    // Banded solves are cheap to launch; small coarse levels still beat the serial CPU reference.
    if (o.compute.equals("auto") && bands.total() < 200_000) return false;
    try {
      GpuSgm.Device device = GpuSgm.selectedDevice(o);
      int group = GpuSgm.selectedGroup(o);
      long total = bands.total();
      boolean fits =
          deviceBytes(o, w, h, total) <= GpuSgm.budget(device, o)
              && total <= 8 * bankCells(o, device)
              && 24L * w * h <= device.allocation()
              && (2L * Math.max(1, bands.maxCount()) + 2) * 4 <= device.localMemory();
      if (!fits && o.compute.equals("gpu"))
        throw new IllegalArgumentException(
            "Packed banded SGM exceeds the GPU budget ("
                + deviceBytes(o, w, h, total) / 1048576
                + " MiB)");
      return fits;
    } catch (RuntimeException e) {
      if (o.compute.equals("gpu")) throw e;
      return false;
    }
  }

  /**
   * Cells per allocation bank. Packed costs and sums are split over up to 8 banks so a volume can
   * exceed the device's single-allocation limit (CL_DEVICE_MAX_MEM_ALLOC_SIZE, typically 1/4 of
   * VRAM). Indices stay below 2^31, so device bank arithmetic is 32-bit.
   */
  static long bankCells(Options o, GpuSgm.Device device) {
    Integer forced = Integer.getInteger("sgm.test.bandedBankCells");
    if (forced != null) return forced;
    long perBank = device.allocation() / Math.max(2, StereoCost.aggregateBytes(o));
    return Math.max(1, Math.min(perBank, (Integer.MAX_VALUE - 8) / 8));
  }

  /** One-shot solve (tests); production code holds a {@link Session} per pair. */
  static int[] summary(
      Options o,
      long[][] a,
      long[][] b,
      byte[] grayA,
      byte[] grayB,
      boolean[] ta,
      boolean[] tb,
      int w,
      int h,
      boolean reverse,
      DisparityBands bands) {
    // Session images are always stored as (left, right); reverse solves swap them back.
    try (Session s =
        reverse
            ? new Session(o, b, a, grayB, grayA, tb, ta, w, h)
            : new Session(o, a, b, grayA, grayB, ta, tb, w, h)) {
      return s.summary(o, reverse, bands);
    }
  }

  static final class Session implements AutoCloseable {
    private final GpuSgm.Compute cl;
    private final Arena arena = Arena.ofConfined();
    private final int w, h, n, words;
    private final MemorySegment descriptorsLeft, descriptorsRight, maskLeft, maskRight, grayLeft, grayRight;
    private final MemorySegment low, count, offset, out, host;
    private final MemorySegment bandHost;
    private final boolean queuedUploads;
    private final MemorySegment[] cost = new MemorySegment[8], sum = new MemorySegment[8];
    private long capacity;
    private final long localMemory, bank, budget;
    private final MemorySegment costs, paths, winners;

    /** Uploads the left/right inputs; a and b are left and right in the forward direction. */
    Session(
        Options o,
        long[][] a,
        long[][] b,
        byte[] grayA,
        byte[] grayB,
        boolean[] ta,
        boolean[] tb,
        int w,
        int h) {
      this.w = w;
      this.h = h;
      queuedUploads = o.gpuQueuedUploads;
      n = w * h;
      words = a.length;
      GpuSgm.Device device = GpuSgm.selectedDevice(o);
      localMemory = device.localMemory();
      bank = bankCells(o, device);
      budget = GpuSgm.budget(device, o);
      cl = new GpuSgm.Compute(o, source(o.matchingCost.equals("zncc"), StereoCost.aggregateBytes(o), StereoCost.gpuSource(o)));
      try {
        descriptorsLeft = descriptors(a);
        descriptorsRight = descriptors(b);
        maskLeft = upload(arena.allocateFrom(JAVA_BYTE, mask(ta)));
        maskRight = upload(arena.allocateFrom(JAVA_BYTE, mask(tb)));
        grayLeft = upload(arena.allocateFrom(JAVA_BYTE, grayA));
        grayRight = upload(arena.allocateFrom(JAVA_BYTE, grayB));
        low = cl.buffer(4L * n);
        count = cl.buffer(4L * n);
        offset = cl.buffer(4L * n);
        out = cl.buffer(24L * n);
        host = arena.allocate(24L * n);
        bandHost = arena.allocate(12L * n, 4);
        costs = cl.kernel("bcosts");
        paths = cl.kernel("bpaths");
        winners = cl.kernel("bwinners");
      } catch (RuntimeException e) {
        close();
        throw e;
      }
    }

    private MemorySegment descriptors(long[][] d) {
      MemorySegment native_ = arena.allocate((long) n * d.length * 8);
      for (int word = 0; word < d.length; word++)
        MemorySegment.copy(MemorySegment.ofArray(d[word]), 0, native_, (long) word * n * 8, (long) n * 8);
      return upload(native_);
    }

    private MemorySegment upload(MemorySegment data) {
      MemorySegment buffer = cl.buffer(Math.max(8, data.byteSize()));
      if (data.byteSize() > 0) write(buffer, data);
      return buffer;
    }

    private void write(MemorySegment buffer, MemorySegment data) {
      if (queuedUploads) cl.writeQueued(buffer, 0, data);
      else cl.transfer(buffer, 0, data, false);
    }

    private static byte[] mask(boolean[] m) {
      byte[] out = new byte[m.length];
      for (int i = 0; i < m.length; i++) out[i] = (byte) (m[i] ? 1 : 0);
      return out;
    }

    int[] summary(Options o, boolean reverse, DisparityBands bands) {
      long total = Math.max(1, bands.total());
      int cap = Math.max(1, bands.maxCount()), sumBytes = StereoCost.aggregateBytes(o);
      if (total > capacity) {
        if (total > 8 * bank)
          throw new IllegalArgumentException(
              "Packed banded volume of " + total + " cells exceeds 8 allocation banks");
        // 25% headroom only when the complete session (including images) fits.
        long grown = total + total / 4;
        capacity =
            grown <= 8 * bank && deviceBytes(o, w, h, grown) <= budget ? grown : total;
        for (int i = 0; i < 8; i++) {
          cl.releaseBuffer(cost[i]);
          cl.releaseBuffer(sum[i]);
          long cells = Math.max(1, Math.min(bank, capacity - i * bank));
          cost[i] = cl.buffer(Math.max(8, cells * 2));
          sum[i] = cl.buffer(Math.max(8, cells * sumBytes));
        }
      }
      List<Object> costArgs = new ArrayList<>(List.of((Object[]) cost)), sumArgs = new ArrayList<>(List.of((Object[]) sum));
      costArgs.add((int) bank);
      sumArgs.add((int) bank);
      // Persistent native staging survives queued uploads and is only overwritten
      // after the previous blocking result read has completed the in-order queue.
      MemorySegment lows = bandHost.asSlice(0, 4L * n),
          counts = bandHost.asSlice(4L * n, 4L * n),
          offsets = bandHost.asSlice(8L * n, 4L * n);
      MemorySegment.copy(MemorySegment.ofArray(bands.low), 0, lows, 0, 4L * n);
      MemorySegment.copy(MemorySegment.ofArray(bands.count), 0, counts, 0, 4L * n);
      MemorySegment.copy(MemorySegment.ofArray(bands.offsets()), 0, offsets, 0, 4L * n);
      write(low, lows);
      write(count, counts);
      write(offset, offsets);
      MemorySegment da = reverse ? descriptorsRight : descriptorsLeft,
          db = reverse ? descriptorsLeft : descriptorsRight,
          ma = reverse ? maskRight : maskLeft,
          mb = reverse ? maskLeft : maskRight,
          ga = reverse ? grayRight : grayLeft,
          gb = reverse ? grayLeft : grayRight;
      List<Object> args = new ArrayList<>(List.of(da, db, ma, mb, ga, gb, low, count, offset));
      args.addAll(costArgs);
      Collections.addAll(
          args, w, h, o.minDisparity, reverse ? 1 : 0, o.census / 2, words, StereoCost.code(o),
          (int) Math.round(o.censusWeight * 1000), o.adTruncation, o.gradientTruncation);
      cl.run(costs, n, args.toArray());
      int group = cl.group(), lanes = lanes(bands, cap, group, localMemory), per = group / lanes;
      int[][] dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
      for (int direction = 0; direction < o.directions; direction++) {
        int dx = dirs[direction][0], dy = dirs[direction][1];
        int lines = dy == 0 ? h : dx == 0 ? w : w + h - 1;
        long groups = (lines + per - 1) / per;
        args = new ArrayList<>(costArgs);
        args.addAll(sumArgs);
        Collections.addAll(
            args, low, count, offset, ma, ga, w, h, dx, dy, StereoCost.penalty(o.p1, o),
            StereoCost.p2(o), o.sgmEdgeScale, direction == 0 ? 1 : 0, cap, lanes, lines,
            new GpuSgm.Local(4L * (2L * cap * per + 2L * per)));
        cl.run(paths, groups * group, args.toArray());
      }
      args = new ArrayList<>(costArgs);
      args.addAll(sumArgs);
      Collections.addAll(
          args, low, count, offset, out, mb, w, h, o.disparities, o.census / 2, o.minDisparity,
          reverse ? 1 : 0);
      cl.run(winners, n, args.toArray());
      int[] summary = new int[n * 6];
      cl.transfer(out, 0, host, true);
      MemorySegment.copy(host, 0, MemorySegment.ofArray(summary), 0, 24L * n);
      return summary;
    }

    /**
     * Lanes per path line: the power of two near the mean band width (8..32, at most the group),
     * raised until every line's two state rows fit in local memory.
     */
    static int lanes(DisparityBands bands, int cap, int group, long localMemory) {
      long pixels = 0;
      for (int c : bands.count) if (c > 0) pixels++;
      double mean = pixels == 0 ? 1 : bands.total() / (double) pixels;
      int lanes = mean > 24 ? 32 : mean > 8 ? 16 : 8;
      lanes = Math.min(lanes, group);
      while (lanes < group && 4L * (2L * cap + 2) * (group / lanes) > localMemory) lanes *= 2;
      return lanes;
    }

    Object profile() {
      return cl.profile();
    }

    @Override
    public void close() {
      try {
        cl.finish();
        RunTelemetry.gpuProfile("Banded SGM " + w + "x" + h, cl.profile());
      } finally {
        try { cl.close(); } finally { arena.close(); }
      }
    }
  }

  static String source(boolean zncc, int sumBytes) {
    return source(zncc, sumBytes, "");
  }

  /** costExtension: StereoCost.gpuSource(o), empty unless the opt-in census modes are set. */
  static String source(boolean zncc, int sumBytes, String costExtension) {
    return (zncc ? "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n#define ZNCC 1\n" : "")
        + "#pragma OPENCL FP_CONTRACT OFF\n"
        + (sumBytes == 2 ? "typedef ushort SUM;\n" : "typedef int SUM;\n")
        + costExtension
        + KERNELS;
  }

  private static final String KERNELS =
      """
#define CP __global ushort *c0,__global ushort *c1,__global ushort *c2,__global ushort *c3,__global ushort *c4,__global ushort *c5,__global ushort *c6,__global ushort *c7,int CBK
#define SP __global SUM *s0,__global SUM *s1,__global SUM *s2,__global SUM *s3,__global SUM *s4,__global SUM *s5,__global SUM *s6,__global SUM *s7,int SBK
#define CA c0,c1,c2,c3,c4,c5,c6,c7,CBK
#define SA s0,s1,s2,s3,s4,s5,s6,s7,SBK
// Packed cells span up to 8 allocation banks; indices are below 2^31.
int cget(int i,CP){int b=i/CBK;i-=b*CBK;switch(b){case 0:return c0[i];case 1:return c1[i];case 2:return c2[i];case 3:return c3[i];case 4:return c4[i];case 5:return c5[i];case 6:return c6[i];default:return c7[i];}}
void cset(int i,int v,CP){int b=i/CBK;i-=b*CBK;switch(b){case 0:c0[i]=(ushort)v;break;case 1:c1[i]=(ushort)v;break;case 2:c2[i]=(ushort)v;break;case 3:c3[i]=(ushort)v;break;case 4:c4[i]=(ushort)v;break;case 5:c5[i]=(ushort)v;break;case 6:c6[i]=(ushort)v;break;default:c7[i]=(ushort)v;}}
int sget(int i,SP){int b=i/SBK;i-=b*SBK;switch(b){case 0:return s0[i];case 1:return s1[i];case 2:return s2[i];case 3:return s3[i];case 4:return s4[i];case 5:return s5[i];case 6:return s6[i];default:return s7[i];}}
void sset(int i,int v,SP){int b=i/SBK;i-=b*SBK;switch(b){case 0:s0[i]=(SUM)v;break;case 1:s1[i]=(SUM)v;break;case 2:s2[i]=(SUM)v;break;case 3:s3[i]=(SUM)v;break;case 4:s4[i]=(SUM)v;break;case 5:s5[i]=(SUM)v;break;case 6:s6[i]=(SUM)v;break;default:s7[i]=(SUM)v;}}
__kernel void bcosts(__global const ulong *a,__global const ulong *b,__global const uchar *ta,__global const uchar *tb,
 __global const uchar *ga,__global const uchar *gb,__global const int *low,__global const int *count,__global const int *offset,
 CP,int w,int h,int md,int reverse,int r,int words,int mode,int weight,int adcap,int gradcap) {
  int p=get_global_id(0),N=w*h;if(p>=N)return;int n=count[p];if(n==0)return;
  int x=p%w,y=p/w,at=offset[p],lo=low[p];size_t sn=(size_t)N;
  for(int j=0;j<n;j++){
    int disp=md+lo+j;long target=(long)x+(reverse?(long)disp:-(long)disp);int value=512;
    if(x>=r&&x<w-r&&y>=r&&y<h-r&&ta[p]&&target>=r&&target<w-r) {
      int q=y*w+(int)target;
      if(tb[q]) {
#ifdef CENSUS_EXT
        value=ccost(a,b,ta,tb,ga,gb,sn,p,q,w,h,r,words,mode,weight,adcap,gradcap);
#else
        int ham=0;for(int k=0;k<words;k++)ham+=popcount(a[(size_t)k*sn+p]^b[(size_t)k*sn+q]);
        value=ham;int bits=(2*r+1)*(2*r+1)-1;
        if(mode==1||mode==2){int difference,cap;
          if(mode==1){difference=abs((int)ga[p]-(int)gb[q]);cap=adcap;}
          else{int ax=(int)ga[p+1]-ga[p-1],ay=(int)ga[p+w]-ga[p-w],bx=(int)gb[q+1]-gb[q-1],by=(int)gb[q+w]-gb[q-w];difference=abs(ax-bx)+abs(ay-by);cap=gradcap;}
          int den=1000*cap;value=(weight*ham*cap+(1000-weight)*min(difference,cap)*bits+den/2)/den;
        }
#endif
#ifdef ZNCC
        if(mode==3){long sa=0,sb=0,saa=0,sbb=0,sab=0;int cnt=(2*r+1)*(2*r+1);
          for(int dy=-r;dy<=r;dy++)for(int dx=-r;dx<=r;dx++){int av=ga[p+dy*w+dx],bv=gb[q+dy*w+dx];sa+=av;sb+=bv;saa+=av*av;sbb+=bv*bv;sab+=av*bv;}
          long va=cnt*saa-sa*sa,vb=cnt*sbb-sb*sb;
          if(va<=0||vb<=0)value=512;else{double correlation=(double)(cnt*sab-sa*sb)/sqrt((double)va*(double)vb);value=(int)floor((1-fmax(-1.0,fmin(1.0,correlation)))*.5*bits+.5);}
        }
#endif
      }
    }
    cset(at+j,value,CA);
  }
}
void bstart(int line,int w,int h,int dx,int dy,int *x,int *y){
  if(dy==0){*x=dx>0?0:w-1;*y=line;}
  else if(dx==0){*x=line;*y=dy>0?0:h-1;}
  else if(line<w){*x=line;*y=dy>0?0:h-1;}
  else{*x=dx>0?0:w-1;*y=dy>0?line-w+1:h-2-(line-w);}
}
int blength(int line,int lines,int w,int h,int dx,int dy){
  if(line>=lines)return 0;int x,y;bstart(line,w,h,dx,dy,&x,&y);int n=2147483647;
  if(dx>0)n=min(n,w-x);if(dx<0)n=min(n,x+1);if(dy>0)n=min(n,h-y);if(dy<0)n=min(n,y+1);return n;
}
// S lanes per path line, G/S lines per work-group; every line keeps its own continuous state.
__kernel void bpaths(CP,SP,__global const int *low,__global const int *count,__global const int *offset,
 __global const uchar *valid,__global const uchar *gray,int w,int h,int dx,int dy,int p1,int p2,int edgeScale,int clear,int cap,int S,int lines,__local int *state) {
  int tid=get_local_id(0),G=get_local_size(0),lane=tid%S,slot=tid/S,per=G/S,line=get_group_id(0)*per+slot,x=0,y=0;
  int steps=0;for(int s=0;s<per;s++)steps=max(steps,blength(get_group_id(0)*per+s,lines,w,h,dx,dy));
  int length=blength(line,lines,w,h,dx,dy);if(length>0)bstart(line,w,h,dx,dy,&x,&y);
  __local int *prev=state+slot*2*cap,*next=prev+cap,*ms=state+per*2*cap+2*slot;int plo=0,pn=0,pmin=0;
  // Per-line double-buffered atomic minimum (two barriers per step).
  if(lane==0){ms[0]=2147483647;ms[1]=2147483647;}barrier(CLK_LOCAL_MEM_FENCE);
  for(int step=0;step<steps;step++) {
    int alive=step<length,mine=2147483647,lo=0,n=0;
    if(alive){
      int p=y*w+x,px=x-dx,py=y-dy;
      int has=step>0&&px>=0&&px<w&&py>=0&&py<h&&valid[py*w+px]&&pn>0;
      int at=offset[p];lo=low[p];n=count[p];
      int localP2=p2;if(has&&edgeScale>0){int difference=abs((int)gray[p]-(int)gray[py*w+px]);localP2=max(p1+1,(int)((long)p2*edgeScale/(edgeScale+difference)));}
      for(int k=lane;k<n;k+=S){int v=cget(at+k,CA);
        if(has){int j=lo+k-plo,t=pmin+localP2;if(j>=0&&j<pn)t=min(t,prev[j]);if(j-1>=0&&j-1<pn)t=min(t,prev[j-1]+p1);if(j+1>=0&&j+1<pn)t=min(t,prev[j+1]+p1);v+=t-pmin;}
        next[k]=v;mine=min(mine,v);sset(at+k,clear?v:sget(at+k,SA)+v,SA);
      }
    }
    atomic_min(&ms[step&1],mine);barrier(CLK_LOCAL_MEM_FENCE);
    if(alive){pmin=ms[step&1];plo=lo;pn=n;x+=dx;y+=dy;}
    if(lane==0)ms[(step+1)&1]=2147483647;
    barrier(CLK_LOCAL_MEM_FENCE);__local int *tmp=prev;prev=next;next=tmp;
  }
}
int bmatch(__global const uchar *tb,int w,int r,int x,int y,int disp,int reverse){long target=(long)x+(reverse?(long)disp:-(long)disp);return target>=r&&target<w-r&&tb[y*w+(int)target];}
__kernel void bwinners(CP,SP,__global const int *low,__global const int *count,__global const int *offset,
 __global int *out,__global const uchar *tb,int w,int h,int D,int r,int md,int reverse) {
  int p=get_global_id(0);if(p>=w*h)return;
  int best=-1,value=2147483647,second=2147483647,before=-1,after=-1,x=p%w,y=p/w,lo=low[p],n=count[p],at=offset[p];
  if(x>=r&&x<w-r&&y>=r&&y<h-r&&n>0){
    for(int k=0;k<n;k++)if(cget(at+k,CA)!=512&&sget(at+k,SA)<value){best=k;value=sget(at+k,SA);}
    if(best>=0){for(int k=0;k<n;k++)if(abs(k-best)>1&&cget(at+k,CA)!=512)second=min(second,sget(at+k,SA));
      if(best>0&&cget(at+best-1,CA)!=512)before=sget(at+best-1,SA);
      if(best+1<n&&cget(at+best+1,CA)!=512)after=sget(at+best+1,SA);}
  }
  int edge=best>=0&&((best==0&&lo>0&&bmatch(tb,w,r,x,y,md+lo-1,reverse))||(best==n-1&&lo+n<D&&bmatch(tb,w,r,x,y,md+lo+n,reverse)));
  int flags=(n>0&&lo==0&&cget(at,CA)!=512?1:0)|(n>0&&lo+n==D&&cget(at+n-1,CA)!=512?2:0)|(edge?4:0);
  size_t o=(size_t)p*6;out[o]=best<0?-1:lo+best;out[o+1]=value;out[o+2]=second;out[o+3]=before;out[o+4]=after;out[o+5]=flags;
}
""";
}
