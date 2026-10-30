package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.io.*;
import java.lang.foreign.*;
import java.nio.file.*;
import java.util.*;

/** Banked resident or exact strip-streamed SGM. Path state crosses every strip boundary. */
final class GpuStereo {
  record Plan(boolean streamed, int rows, long bankBytes, long deviceBytes, long hostBytes) {}

  private static long bankBytes(GpuSgm.Device d, Options o) {
    return Math.min(d.allocation(), o.gpuBankMb * 1048576L) / 8 * 8;
  }

  static Plan plan(GpuSgm.Device device, int group, int w, int h, int D, Options o) {
    int sumBytes = StereoCost.aggregateBytes(o);
    long n = (long) w * h, bank = bankBytes(device, o), budget = GpuSgm.budget(device, o);
    int words = (o.census * o.census - 1 + 63) / 64;
    long input = n * (16L * words + 4), host = n * (96L + 32 * words);
    long hostBudget =
        Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    if (host > hostBudget) return null;
    if (D < 3
        || D > 4096
        || (2L * D + group) * 4 > device.localMemory()
        || bank < 8
        || n * words * 8 > device.allocation()) return null;
    long fullCells = n * D, resident = input + fullCells * (2 + sumBytes) + n * 24 + 64;
    if (!o.gpuSgm.equals("streamed")
        && resident <= budget
        && fullCells * sumBytes <= bank * 8
        && n * 24 <= device.allocation()) return new Plan(false, h, bank, resident, host);
    if (o.gpuSgm.equals("resident") || (long) w * D * 4 > device.allocation()) return null;
    for (int rows = Math.min(h, o.gpuStripRows); rows >= 1; rows /= 2) {
      long cells = (long) w * rows * D,
          bytes = input + cells * (2 + sumBytes) + (long) w * rows * 24 + 8L * w * D + 64;
      long staging = host + (long) w * rows * ((long) sumBytes * D + 24);
      if (bytes <= budget
          && cells * sumBytes <= bank * 8
          && (long) w * rows * 24 <= device.allocation()
          && staging <= hostBudget) return new Plan(true, rows, bank, bytes, staging);
    }
    return null;
  }

  static int limit(GpuSgm.Device d, int group, int w, int h, Options o) {
    int low = 2, high = 4096;
    while (low < high) {
      int mid = (low + high + 1) / 2;
      if (plan(d, group, w, h, mid, o) != null) low = mid;
      else high = mid - 1;
    }
    return low == 2 ? 0 : low;
  }

  static Object report(int w, int h, Options o) {
    if (o.compute.equals("cpu")) return null;
    try {
      Plan p = plan(GpuSgm.selectedDevice(o), GpuSgm.selectedGroup(o), w, h, o.disparities, o);
      return p == null
          ? null
          : Json.object(
              "storage",
              p.streamed ? "streamed" : "resident_banked",
              "rows",
              p.rows,
              "bank_bytes",
              p.bankBytes,
              "estimated_device_bytes",
              p.deviceBytes,
              "estimated_host_bytes",
              p.hostBytes,
              "aggregate_bits",
              8 * StereoCost.aggregateBytes(o),
              "aggregate_proved_bound",
              StereoCost.aggregateBound(o),
              "cost_generation_passes",
              p.streamed ? o.directions : 1,
              "aggregate_volume_reads",
              p.streamed ? o.directions - 1 : 0,
              "aggregate_volume_writes",
              p.streamed ? o.directions - 1 : 0,
              "aggregate_disk_bytes",
              p.streamed ? (long) StereoCost.aggregateBytes(o) * w * h * o.disparities : 0);
    } catch (RuntimeException e) {
      return Json.object("unavailable", e.getMessage());
    }
  }

  static float[] execute(
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
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary) {
    GpuSgm.Device device = GpuSgm.selectedDevice(o);
    Plan plan = plan(device, GpuSgm.selectedGroup(o), w, h, o.disparities, o);
    if (plan == null)
      throw new IllegalArgumentException("No GPU SGM memory plan fits the requested range");
    try (GpuSgm.Compute cl =
            new GpuSgm.Compute(
                o, kernels(o.matchingCost.equals("zncc"), StereoCost.aggregateBytes(o)));
        Arena arena = Arena.ofConfined()) {
      int n = w * h,
          D = o.disparities,
          rows = plan.rows,
          words = a.length,
          sumBytes = StereoCost.aggregateBytes(o);
      long stripCells = (long) w * rows * D;
      MemorySegment first = arena.allocate((long) n * words * 8),
          second = arena.allocate(first.byteSize());
      for (int word = 0; word < words; word++) {
        MemorySegment.copy(
            MemorySegment.ofArray(a[word]), 0, first, (long) word * n * 8, (long) n * 8);
        MemorySegment.copy(
            MemorySegment.ofArray(b[word]), 0, second, (long) word * n * 8, (long) n * 8);
      }
      MemorySegment ga = upload(cl, first), gb = upload(cl, second);
      byte[] ma = new byte[n], mb = new byte[n];
      for (int i = 0; i < n; i++) {
        ma[i] = (byte) (ta[i] ? 1 : 0);
        mb[i] = (byte) (tb[i] ? 1 : 0);
      }
      MemorySegment gma = upload(cl, arena.allocateFrom(JAVA_BYTE, ma)),
          gmb = upload(cl, arena.allocateFrom(JAVA_BYTE, mb)),
          gra = upload(cl, arena.allocateFrom(JAVA_BYTE, grayA)),
          grb = upload(cl, arena.allocateFrom(JAVA_BYTE, grayB));
      MemorySegment[] cost = banks(cl, stripCells * 2, plan.bankBytes),
          sum = banks(cl, stripCells * sumBytes, plan.bankBytes);
      MemorySegment out = cl.buffer((long) w * rows * 24),
          carry0 = cl.buffer(plan.streamed ? 4L * w * D : 8),
          carry1 = cl.buffer(plan.streamed ? 4L * w * D : 8);
      MemorySegment costs = cl.kernel("costs"),
          paths = cl.kernel("paths"),
          winners = cl.kernel("winners");
      System.out.printf(
          Locale.ROOT,
          "GPU SGM storage: %s; %d-row working set, %.1f MiB device; cost=%s; P1/P2=%d/%d%n",
          plan.streamed ? "streamed, continuous paths" : "resident allocation banks",
          rows,
          plan.deviceBytes / 1048576.,
          o.matchingCost,
          StereoCost.penalty(o.p1, o),
          StereoCost.p2(o));
      float[] result = new float[n];
      Arrays.fill(result, Float.NaN);
      Options storage = o.copy();
      storage.tileSize = Math.max(1024, Math.min(262144, w));
      storage.disparities = 1;
      if (plan.streamed) {
        Files.createDirectories(o.output);
        long disk = (long) sumBytes * n * D,
            available = Files.getFileStore(o.output).getUsableSpace();
        if (available > 0 && disk + 16777216L > available)
          throw new IllegalArgumentException(
              "Streamed GPU SGM needs "
                  + disk / 1048576
                  + " MiB temporary aggregate storage; insufficient disk space");
      }
      MemorySegment host = arena.allocate(plan.streamed ? stripCells * sumBytes : 8),
          summary = arena.allocate((long) w * rows * 24);
      try (PagedVolume aggregate =
          plan.streamed
              ? new PagedVolume((long) n * D, sumBytes, 0, true, storage, "gpu-sgm-aggregate-")
              : null) {
        int[][] directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
        for (int direction = 0; direction < o.directions; direction++) {
          int dx = directions[direction][0], dy = directions[direction][1];
          MemorySegment input = carry0, output = carry1;
          int stripes = (h + rows - 1) / rows;
          for (int ordinal = 0; ordinal < stripes; ordinal++) {
            int stripe = dy < 0 ? stripes - 1 - ordinal : ordinal,
                y0 = stripe * rows,
                actual = Math.min(rows, h - y0);
            if (plan.streamed || direction == 0)
              cost(
                  cl,
                  costs,
                  ga,
                  gb,
                  gma,
                  gmb,
                  gra,
                  grb,
                  cost,
                  sum,
                  plan.bankBytes,
                  w,
                  h,
                  D,
                  o,
                  reverse,
                  words,
                  y0,
                  actual);
            if (plan.streamed && direction > 0) {
              MemorySegment data = host.asSlice(0, (long) sumBytes * w * actual * D);
              aggregate.transfer((long) y0 * w * D, data, false);
              transferBanks(cl, sum, plan.bankBytes, data, false);
            }
            List<Object> arguments = new ArrayList<>();
            bankArguments(arguments, cost, sum, plan.bankBytes, sumBytes);
            Collections.addAll(
                arguments,
                gma,
                gra,
                w,
                h,
                D,
                dx,
                dy,
                StereoCost.penalty(o.p1, o),
                StereoCost.p2(o),
                o.sgmEdgeScale,
                y0,
                actual,
                plan.streamed ? 1 : 0,
                ordinal > 0 ? 1 : 0,
                input,
                output,
                new GpuSgm.Local(4L * (2 * D + cl.group())));
            long lines = dy == 0 ? actual : dx == 0 ? w : w + actual - 1;
            cl.run(paths, lines * cl.group(), arguments.toArray());
            if (plan.streamed && direction == o.directions - 1) {
              winners(
                  cl,
                  winners,
                  cost,
                  sum,
                  plan.bankBytes,
                  out,
                  summary,
                  w,
                  h,
                  D,
                  o,
                  y0,
                  actual,
                  result,
                  stats,
                  reasons,
                  boundary);
            } else if (plan.streamed) {
              MemorySegment data = host.asSlice(0, (long) sumBytes * w * actual * D);
              transferBanks(cl, sum, plan.bankBytes, data, true);
              aggregate.transfer((long) y0 * w * D, data, true);
            }
            MemorySegment swap = input;
            input = output;
            output = swap;
          }
        }
        if (!plan.streamed)
          winners(
              cl,
              winners,
              cost,
              sum,
              plan.bankBytes,
              out,
              summary,
              w,
              h,
              D,
              o,
              0,
              h,
              result,
              stats,
              reasons,
              boundary);
      }
      return result;
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private static void winners(
      GpuSgm.Compute cl,
      MemorySegment kernel,
      MemorySegment[] cost,
      MemorySegment[] sum,
      long bank,
      MemorySegment out,
      MemorySegment summary,
      int w,
      int h,
      int D,
      Options o,
      int y0,
      int actual,
      float[] result,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary) {
    List<Object> args = new ArrayList<>();
    bankArguments(args, cost, sum, bank, StereoCost.aggregateBytes(o));
    Collections.addAll(args, out, w, h, D, o.census / 2, y0, actual);
    cl.run(kernel, (long) w * actual, args.toArray());
    MemorySegment data = summary.asSlice(0, (long) w * actual * 24);
    cl.transfer(out, 0, data, true);
    select(data, y0 * w, w * actual, D, o, result, stats, reasons, boundary);
  }

  private static MemorySegment upload(GpuSgm.Compute cl, MemorySegment data) {
    MemorySegment b = cl.buffer(data.byteSize());
    cl.transfer(b, 0, data, false);
    return b;
  }

  private static MemorySegment[] banks(GpuSgm.Compute cl, long total, long bank) {
    if (total > bank * 8)
      throw new IllegalArgumentException("SGM volume exceeds allocation-bank capacity");
    MemorySegment[] b = new MemorySegment[8];
    for (int i = 0; i < 8; i++) b[i] = cl.buffer(Math.max(8, Math.min(bank, total - i * bank)));
    return b;
  }

  private static void bankArguments(
      List<Object> arguments, MemorySegment[] cost, MemorySegment[] sum, long bytes, int sumBytes) {
    Collections.addAll(arguments, cost);
    Collections.addAll(arguments, sum);
    arguments.add((int) (bytes / 2));
    arguments.add((int) (bytes / sumBytes));
  }

  private static void transferBanks(
      GpuSgm.Compute cl, MemorySegment[] bank, long bytes, MemorySegment data, boolean read) {
    for (long offset = 0; offset < data.byteSize(); offset += bytes)
      cl.transfer(
          bank[(int) (offset / bytes)],
          0,
          data.asSlice(offset, Math.min(bytes, data.byteSize() - offset)),
          read);
  }

  private static void cost(
      GpuSgm.Compute cl,
      MemorySegment kernel,
      MemorySegment a,
      MemorySegment b,
      MemorySegment ma,
      MemorySegment mb,
      MemorySegment grayA,
      MemorySegment grayB,
      MemorySegment[] cost,
      MemorySegment[] sum,
      long bank,
      int w,
      int h,
      int D,
      Options o,
      boolean reverse,
      int words,
      int y0,
      int rows) {
    List<Object> args = new ArrayList<>();
    Collections.addAll(args, a, b, ma, mb, grayA, grayB);
    bankArguments(args, cost, sum, bank, StereoCost.aggregateBytes(o));
    Collections.addAll(
        args,
        w,
        h,
        D,
        o.minDisparity,
        reverse ? 1 : 0,
        o.census / 2,
        words,
        y0,
        rows,
        StereoCost.code(o),
        (int) Math.round(o.censusWeight * 1000),
        o.adTruncation,
        o.gradientTruncation);
    cl.run(kernel, (long) w * rows * D, args.toArray());
  }

  private static void select(
      MemorySegment host,
      int first,
      int count,
      int D,
      Options o,
      float[] result,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary) {
    for (int j = 0; j < count; j++) {
      long base = j * 24L;
      int p = first + j,
          best = host.get(JAVA_INT, base),
          value = host.get(JAVA_INT, base + 4),
          second = host.get(JAVA_INT, base + 8),
          before = host.get(JAVA_INT, base + 12),
          after = host.get(JAVA_INT, base + 16);
      if (best < 0) continue;
      boolean full = host.get(JAVA_INT, base + 20) == 3;
      if (full) {
        stats.fullIntervalPixels++;
        if (boundary != null) boundary[p] = 4;
      }
      if (best == 0 || best == D - 1) {
        stats.boundaryRejected++;
        if (reasons != null) reasons[p] = 2;
        if (full && best == 0) {
          stats.lowerBoundaryRejected++;
          if (boundary != null) boundary[p] |= 1;
        }
        if (full && best == D - 1) {
          stats.upperBoundaryRejected++;
          if (boundary != null) boundary[p] |= 2;
        }
        continue;
      }
      if (second != Integer.MAX_VALUE && second - value <= Math.max(1, value * o.uniqueness)) {
        stats.ambiguityRejected++;
        if (reasons != null) reasons[p] = 3;
        continue;
      }
      double offset = 0;
      if (before >= 0 && after >= 0) {
        double curvature = (double) before + after - 2. * value;
        if (curvature > 0)
          offset = Math.max(-.5, Math.min(.5, .5 * (before - (double) after) / curvature));
      }
      result[p] = (float) (o.minDisparity + best + offset);
      stats.selected++;
      if (reasons != null) reasons[p] = 0;
    }
  }

  private static String kernels(boolean zncc, int sumBytes) {
    return (zncc ? "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n#define ZNCC 1\n" : "")
        + "#pragma OPENCL FP_CONTRACT OFF\n"
        + (sumBytes == 2 ? "typedef ushort SUM;\n" : "typedef int SUM;\n")
        + KERNELS;
  }

  private static final String KERNELS =
      """
#define CARGS __global ushort *c0,__global ushort *c1,__global ushort *c2,__global ushort *c3,__global ushort *c4,__global ushort *c5,__global ushort *c6,__global ushort *c7
#define SARGS __global SUM *s0,__global SUM *s1,__global SUM *s2,__global SUM *s3,__global SUM *s4,__global SUM *s5,__global SUM *s6,__global SUM *s7
#define CV c0,c1,c2,c3,c4,c5,c6,c7
#define SV s0,s1,s2,s3,s4,s5,s6,s7
int cr(ulong at,int B,CARGS){int bank=(int)(at/(ulong)B),i=(int)(at%(ulong)B);switch(bank){case 0:return c0[i];case 1:return c1[i];case 2:return c2[i];case 3:return c3[i];case 4:return c4[i];case 5:return c5[i];case 6:return c6[i];default:return c7[i];}}
void cw(ulong at,int B,int v,CARGS){int bank=(int)(at/(ulong)B),i=(int)(at%(ulong)B);switch(bank){case 0:c0[i]=v;break;case 1:c1[i]=v;break;case 2:c2[i]=v;break;case 3:c3[i]=v;break;case 4:c4[i]=v;break;case 5:c5[i]=v;break;case 6:c6[i]=v;break;default:c7[i]=v;}}
int sr(ulong at,int B,SARGS){int bank=(int)(at/(ulong)B),i=(int)(at%(ulong)B);switch(bank){case 0:return s0[i];case 1:return s1[i];case 2:return s2[i];case 3:return s3[i];case 4:return s4[i];case 5:return s5[i];case 6:return s6[i];default:return s7[i];}}
void sw(ulong at,int B,int v,SARGS){int bank=(int)(at/(ulong)B),i=(int)(at%(ulong)B);switch(bank){case 0:s0[i]=v;break;case 1:s1[i]=v;break;case 2:s2[i]=v;break;case 3:s3[i]=v;break;case 4:s4[i]=v;break;case 5:s5[i]=v;break;case 6:s6[i]=v;break;default:s7[i]=v;}}
__kernel void costs(__global const ulong *a,__global const ulong *b,__global const uchar *ta,__global const uchar *tb,
 __global const uchar *ga,__global const uchar *gb,CARGS,SARGS,int CB,int SB,int w,int h,int D,int md,int reverse,int r,int words,int y0,int rows,int mode,int weight,int adcap,int gradcap) {
  size_t i=get_global_id(0),n=(size_t)w*h;if(i>=(size_t)w*rows*D)return;
  int localp=i/D,p=localp+y0*w,k=i%D,x=p%w,y=p/w,disp=md+k;
  long target=(long)x+(reverse?(long)disp:-(long)disp);int value=512;
  if(x>=r&&x<w-r&&y>=r&&y<h-r&&ta[p]&&target>=r&&target<w-r) {
    int q=y*w+(int)target;
    if(tb[q]) {
      int ham=0;for(int j=0;j<words;j++)ham+=popcount(a[(size_t)j*n+p]^b[(size_t)j*n+q]);
      value=ham;int bits=(2*r+1)*(2*r+1)-1;
      if(mode==1||mode==2){int difference,cap;
        if(mode==1){difference=abs((int)ga[p]-(int)gb[q]);cap=adcap;}
        else{int ax=(int)ga[p+1]-ga[p-1],ay=(int)ga[p+w]-ga[p-w],bx=(int)gb[q+1]-gb[q-1],by=(int)gb[q+w]-gb[q-w];difference=abs(ax-bx)+abs(ay-by);cap=gradcap;}
        int den=1000*cap;value=(weight*ham*cap+(1000-weight)*min(difference,cap)*bits+den/2)/den;
      }
#ifdef ZNCC
      if(mode==3){long sa=0,sb=0,saa=0,sbb=0,sab=0;int count=(2*r+1)*(2*r+1);
        for(int dy=-r;dy<=r;dy++)for(int dx=-r;dx<=r;dx++){int av=ga[p+dy*w+dx],bv=gb[q+dy*w+dx];sa+=av;sb+=bv;saa+=av*av;sbb+=bv*bv;sab+=av*bv;}
        long va=count*saa-sa*sa,vb=count*sbb-sb*sb;
        if(va<=0||vb<=0)value=512;else{double correlation=(double)(count*sab-sa*sb)/sqrt((double)va*(double)vb);value=(int)floor((1-fmax(-1.0,fmin(1.0,correlation)))*.5*bits+.5);}
      }
#endif
    }
  }
  cw(i,CB,value,CV);sw(i,SB,0,SV);
}
__kernel void paths(CARGS,SARGS,int CB,int SB,__global const uchar *valid,__global const uchar *gray,int w,int h,int D,int dx,int dy,int p1,int p2,int edgeScale,
 int y0,int rows,int streamed,int carry,__global const int *incoming,__global int *outgoing,__local int *state) {
  int line=get_group_id(0),tid=get_local_id(0),L=get_local_size(0),x,y;
  if(dy==0){x=dx>0?0:w-1;y=line;}
  else if(dx==0){x=line;y=dy>0?0:rows-1;}
  else if(line<w){x=line;y=dy>0?0:rows-1;}
  else{x=dx>0?0:w-1;y=dy>0?line-w+1:rows-2-(line-w);}
  __local int *prev=state,*next=state+D,*mins=state+2*D;int step=0;
  while(x>=0&&x<w&&y>=0&&y<rows) {
    int globalp=(y+y0)*w+x,px=x-dx,py=y+y0-dy;
    int has=px>=0&&px<w&&py>=0&&py<h&&(step>0||(streamed&&carry&&dy!=0))&&valid[py*w+px];
    if(step==0&&has)for(int k=tid;k<D;k+=L)prev[k]=incoming[(size_t)px*D+k];
    barrier(CLK_LOCAL_MEM_FENCE);
    int minimum=2147483647;if(has)for(int k=tid;k<D;k+=L)minimum=min(minimum,prev[k]);
    mins[tid]=minimum;barrier(CLK_LOCAL_MEM_FENCE);
    for(int stride=L/2;stride>0;stride/=2){if(tid<stride)mins[tid]=min(mins[tid],mins[tid+stride]);barrier(CLK_LOCAL_MEM_FENCE);}
    minimum=mins[0];
    int localP2=p2;if(has&&edgeScale>0){int difference=abs((int)gray[globalp]-(int)gray[py*w+px]);localP2=max(p1+1,(int)((long)p2*edgeScale/(edgeScale+difference)));}
    for(int k=tid;k<D;k+=L){ulong at=((ulong)y*w+x)*D+k;int v=cr(at,CB,CV);
      if(has){int t=min(prev[k],minimum+localP2);if(k>0)t=min(t,prev[k-1]+p1);if(k+1<D)t=min(t,prev[k+1]+p1);v+=t-minimum;}
      next[k]=v;sw(at,SB,sr(at,SB,SV)+v,SV);
      if(streamed&&dy!=0&&y==(dy>0?rows-1:0))outgoing[(size_t)x*D+k]=v;
    }
    barrier(CLK_LOCAL_MEM_FENCE);__local int *tmp=prev;prev=next;next=tmp;x+=dx;y+=dy;step++;
  }
}
__kernel void winners(CARGS,SARGS,int CB,int SB,__global int *out,int w,int h,int D,int r,int y0,int rows) {
  int p=get_global_id(0);if((size_t)p>=(size_t)w*rows)return;
  int best=-1,value=2147483647,second=2147483647,before=-1,after=-1,x=p%w,y=p/w+y0;ulong base=(ulong)p*D;
  if(x>=r&&x<w-r&&y>=r&&y<h-r){
    for(int k=0;k<D;k++)if(cr(base+k,CB,CV)!=512&&sr(base+k,SB,SV)<value){best=k;value=sr(base+k,SB,SV);}
    if(best>=0){for(int k=0;k<D;k++)if(abs(k-best)>1&&cr(base+k,CB,CV)!=512)second=min(second,sr(base+k,SB,SV));
      if(best>0&&cr(base+best-1,CB,CV)!=512)before=sr(base+best-1,SB,SV);
      if(best+1<D&&cr(base+best+1,CB,CV)!=512)after=sr(base+best+1,SB,SV);}
  }
  size_t at=(size_t)p*6;out[at]=best;out[at+1]=value;out[at+2]=second;out[at+3]=before;out[at+4]=after;
  out[at+5]=(cr(base,CB,CV)!=512?1:0)|(cr(base+D-1,CB,CV)!=512?2:0);
}
""";
}
