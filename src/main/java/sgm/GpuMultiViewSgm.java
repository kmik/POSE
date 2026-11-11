package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.*;

/**
 * OpenCL solve of one camera's multi-view SGM ({@link MultiViewSgm}): a multi-view cost kernel fills
 * GpuBandedStereo's banked packed cost volume, GpuBandedStereo's unchanged {@code bpaths} kernel
 * aggregates the 4/8 continuous paths, and a winners kernel summarizes. Results equal
 * {@link MultiViewSgm#cpuSummary} exactly (checked on first use).
 *
 * <p>Device memory per camera is bounded (packed volume + inputs, {@link #deviceBytes}) and reserved
 * from the GPU memory pool when one is installed.
 */
final class GpuMultiViewSgm {
  private GpuMultiViewSgm() {}

  /** Inputs: reference census/mask/gray, bands, summary, neighbour canvases (census + valid), table. */
  static long inputBytes(MultiViewSgm.Plan plan) {
    long n = (long) plan.ref.w * plan.ref.h;
    return n * (8 + 1 + 1 + 12 + 24) + plan.canvasPixels() * 9 + 4L * plan.table.length + 16L * plan.views.length + 64;
  }

  static long deviceBytes(MultiViewSgm mv, MultiViewSgm.Plan plan, long cells) {
    return cells * (2L + mv.sumBytes()) + inputBytes(plan);
  }

  static boolean use(Options o, MultiViewSgm mv, MultiViewSgm.Plan plan, DisparityBands bands) {
    if (o.compute.equals("cpu") || Boolean.getBoolean("sgm.cpuMvsgm")) return false;
    try {
      GpuSgm.Device device = GpuSgm.selectedDevice(o);
      long total = bands.total(), n = (long) plan.ref.w * plan.ref.h;
      boolean fits =
          deviceBytes(mv, plan, total) <= GpuSgm.budget(device, o)
              && total <= 8 * GpuBandedStereo.bankCells(o, device)
              && 24L * n <= device.allocation()
              && plan.canvasPixels() * 8 <= device.allocation()
              && (2L * Math.max(1, bands.maxCount()) + 2) * 4 <= device.localMemory();
      if (!fits && o.compute.equals("gpu"))
        throw new IllegalArgumentException(
            "mvsgm volume exceeds the GPU budget (" + deviceBytes(mv, plan, total) / 1048576 + " MiB)");
      return fits;
    } catch (RuntimeException e) {
      if (o.compute.equals("gpu")) throw e;
      return false;
    }
  }

  static final class Session implements AutoCloseable {
    final MultiViewSgm.Plan plan;
    private final MultiViewSgm mv;
    private final Options o;
    private final GpuSgm.Compute cl;
    private final Arena arena = Arena.ofConfined();
    private final int w, h, n;
    private final MemorySegment refCensus, refUse, refGray, viewCensus, viewValid, viewMeta, table;
    private final MemorySegment metadata, out, host, bandHost;
    private final MemorySegment[] cost = new MemorySegment[8], sum = new MemorySegment[8];
    private final MemorySegment costs, paths, winners;
    private final long bank, budget, localMemory;
    private final GpuMemoryPool.Reservation reservation;
    private long capacity;
    private int[] summaryBuffer;

    Session(Options o, MultiViewSgm mv, MultiViewSgm.Plan plan, long reserveBytes) {
      reservation = GpuMemoryPool.reserve(reserveBytes);
      this.o = o;
      this.mv = mv;
      this.plan = plan;
      w = plan.ref.w;
      h = plan.ref.h;
      n = w * h;
      GpuSgm.Device device = GpuSgm.selectedDevice(o);
      bank = GpuBandedStereo.bankCells(o, device);
      budget = GpuSgm.budget(device, o);
      localMemory = device.localMemory();
      try {
        cl = new GpuSgm.Compute(o, GpuBandedStereo.source(false, mv.sumBytes(), "") + KERNELS);
      } catch (RuntimeException | Error e) {
        reservation.close();
        throw e;
      }
      try {
        refCensus = upload(arena.allocateFrom(JAVA_LONG, plan.ref.census));
        refUse = upload(arena.allocateFrom(JAVA_BYTE, bits(plan.ref.use)));
        refGray = upload(arena.allocateFrom(JAVA_BYTE, plan.ref.gray));
        int V = plan.views.length;
        long canvas = plan.canvasPixels();
        MemorySegment census = arena.allocate(8 * canvas, 8), valid = arena.allocate(canvas);
        int[] meta = new int[4 * V];
        long at = 0;
        for (int v = 0; v < V; v++) {
          var view = plan.views[v];
          int pixels = view.w * view.h;
          MemorySegment.copy(MemorySegment.ofArray(view.census), 0, census, 8 * at, 8L * pixels);
          MemorySegment.copy(MemorySegment.ofArray(bits(view.valid)), 0, valid, at, pixels);
          meta[4 * v] = (int) at;
          meta[4 * v + 1] = view.w;
          meta[4 * v + 2] = view.h;
          at += pixels;
        }
        viewCensus = upload(census);
        viewValid = upload(valid);
        viewMeta = upload(arena.allocateFrom(JAVA_INT, meta));
        table = upload(arena.allocateFrom(JAVA_FLOAT, plan.table));
        metadata = cl.buffer(12L * n);
        out = cl.buffer(24L * n);
        host = arena.allocate(24L * n);
        bandHost = arena.allocate(12L * n, 4);
        costs = cl.kernel("mvcosts");
        paths = cl.kernel("bpaths");
        winners = cl.kernel("mvwinners");
      } catch (RuntimeException | Error e) {
        try {
          close();
        } catch (RuntimeException | Error suppressed) {
          e.addSuppressed(suppressed);
        }
        throw e;
      }
    }

    private static byte[] bits(boolean[] m) {
      byte[] b = new byte[m.length];
      for (int i = 0; i < m.length; i++) b[i] = (byte) (m[i] ? 1 : 0);
      return b;
    }

    private MemorySegment upload(MemorySegment data) {
      MemorySegment buffer = cl.buffer(Math.max(8, data.byteSize()));
      if (data.byteSize() > 0) cl.transfer(buffer, 0, data, false);
      return buffer;
    }

    int[] summary(DisparityBands bands) {
      long total = Math.max(1, bands.total());
      int cap = Math.max(1, bands.maxCount()), sumBytes = mv.sumBytes();
      if (total > capacity) {
        if (total > 8 * bank)
          throw new IllegalArgumentException("mvsgm packed volume of " + total + " cells exceeds 8 allocation banks");
        capacity = total;
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
      MemorySegment.copy(MemorySegment.ofArray(bands.low), 0, bandHost, 0, 4L * n);
      MemorySegment.copy(MemorySegment.ofArray(bands.count), 0, bandHost, 4L * n, 4L * n);
      MemorySegment.copy(MemorySegment.ofArray(bands.offsets()), 0, bandHost, 8L * n, 4L * n);
      cl.transfer(metadata, 0, bandHost, false);
      List<Object> args = new ArrayList<>(List.of(refCensus, refUse, viewCensus, viewValid, viewMeta, table, metadata));
      args.addAll(costArgs);
      Collections.addAll(args, w, h, plan.views.length, mv.k, mv.penalty());
      cl.run(costs, n, args.toArray());
      int group = cl.group(), lanes = GpuBandedStereo.Session.lanes(bands, cap, group, localMemory), per = group / lanes;
      int[][] dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
      int p1 = mv.p1(), p2 = mv.p2();
      for (int direction = 0; direction < o.directions; direction++) {
        int dx = dirs[direction][0], dy = dirs[direction][1];
        int lines = dy == 0 ? h : dx == 0 ? w : w + h - 1;
        long groups = (lines + per - 1) / per;
        args = new ArrayList<>(costArgs);
        args.addAll(sumArgs);
        Collections.addAll(
            args, metadata, refUse, refGray, w, h, dx, dy, p1, p2, o.sgmEdgeScale,
            direction == 0 ? 1 : 0, cap, lanes, lines, new GpuSgm.Local(4L * (2L * cap * per + 2L * per)));
        cl.run(paths, groups * group, args.toArray());
      }
      args = new ArrayList<>(costArgs);
      args.addAll(sumArgs);
      Collections.addAll(args, metadata, out, w, h, plan.count);
      cl.run(winners, n, args.toArray());
      if (summaryBuffer == null) summaryBuffer = new int[n * 6];
      cl.transfer(out, 0, host, true);
      MemorySegment.copy(host, 0, MemorySegment.ofArray(summaryBuffer), 0, 24L * n);
      return summaryBuffer;
    }

    @Override
    public void close() {
      try {
        cl.finish();
        RunTelemetry.gpuProfile("Multi-view SGM " + w + "x" + h, cl.profile());
      } finally {
        try {
          cl.close();
        } finally {
          try {
            arena.close();
          } finally {
            reservation.close();
          }
        }
      }
    }
  }

  // Appended to GpuBandedStereo.source(): uses its CP/SP bank macros, cget/cset/sget and bpaths.
  private static final String KERNELS =
      """
__kernel void mvcosts(__global const ulong *rc,__global const uchar *ruse,__global const ulong *vc,__global const uchar *vv,
 __global const int *vm,__global const float *tab,__global const int *bands,CP,int w,int h,int V,int k,int penalty) {
  __global const int *low=bands,*count=bands+w*h,*offset=bands+2*w*h;
  int p=get_global_id(0);if(p>=w*h)return;int n=count[p];if(n==0)return;
  int x=p%w,y=p/w,at=offset[p],lo=low[p];
  if(!ruse[p]){for(int j=0;j<n;j++)cset(at+j,512,CA);return;}
  ulong c=rc[p];float fxp=(float)x,fyp=(float)y;int each[9];int kk=min(k,V);
  for(int j=0;j<n;j++){
    int l=lo+j,valid=0;
    for(int v=0;v<V;v++){
      int t=3*(l*V+v),cw=vm[4*v+1],ch=vm[4*v+2];float a=tab[t];
      float sx=fxp*a+tab[t+1],sy=fyp*a+tab[t+2];
      if(!(sx>=0.0f)||!(sy>=0.0f)||!(sx<(float)(cw-1))||!(sy<(float)(ch-1)))continue;
      float flx=floor(sx),fly=floor(sy);int ix=(int)flx,iy=(int)fly;
      int b=vm[4*v]+iy*cw+ix;
      if(!vv[b]||!vv[b+1]||!vv[b+cw]||!vv[b+cw+1])continue;
      float fx=sx-flx,fy=sy-fly;
      float h00=(float)popcount(c^vc[b]),h10=(float)popcount(c^vc[b+1]),h01=(float)popcount(c^vc[b+cw]),h11=(float)popcount(c^vc[b+cw+1]);
      float top=(1.0f-fx)*h00+fx*h10,bottom=(1.0f-fx)*h01+fx*h11;
      float hv=(1.0f-fy)*top+fy*bottom;
      int hh=(int)floor(hv*8.0f+0.5f);
      int i=valid++;while(i>0&&each[i-1]>hh){each[i]=each[i-1];i--;}each[i]=hh;
    }
    int value=512;
    if(valid>0){int s=0;for(int i=0;i<kk;i++)s+=i<valid?each[i]:penalty;value=(s+kk/2)/kk;}
    cset(at+j,value,CA);
  }
}
__kernel void mvwinners(CP,SP,__global const int *bands,__global int *out,int w,int h,int L) {
  __global const int *low=bands,*count=bands+w*h,*offset=bands+2*w*h;
  int p=get_global_id(0);if(p>=w*h)return;
  int best=-1,value=2147483647,second=2147483647,before=-1,after=-1,lo=low[p],n=count[p],at=offset[p];
  for(int j=0;j<n;j++)if(cget(at+j,CA)!=512&&sget(at+j,SA)<value){best=j;value=sget(at+j,SA);}
  if(best>=0){for(int j=0;j<n;j++)if(abs(j-best)>1&&cget(at+j,CA)!=512)second=min(second,sget(at+j,SA));
    if(best>0&&cget(at+best-1,CA)!=512)before=sget(at+best-1,SA);
    if(best+1<n&&cget(at+best+1,CA)!=512)after=sget(at+best+1,SA);}
  int flags=0;if(best>=0){int label=lo+best;flags=(best==0&&lo>0?1:0)|(best==n-1&&lo+n<L?2:0)|(label==0||label==L-1?4:0);}
  size_t o=(size_t)p*6;out[o]=best<0?-1:lo+best;out[o+1]=value;out[o+2]=second;out[o+3]=before;out[o+4]=after;out[o+5]=flags;
}
""";
}
