package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.*;

/** Batched FP64 fusion. Exact float/offset evidence stays resident; no lossy depth conversion. */
final class GpuFusion implements AutoCloseable {
  static final int MAX_BATCH = 4096;
  static final String[] STATUS = {
    "accepted",
    "depth_consensus_rejected",
    "independent_support_rejected",
    "refinement_reprojection_rejected",
    "refinement_support_rejected",
    "depth_uncertainty_rejected"
  };
  private final GpuSgm.Compute cl;
  private final int images, pairs, endpoints, batch, bankCells, required, refine;
  private final MemorySegment[] banks = new MemorySegment[8];
  private MemorySegment camera, dimensions, mapInfo, starts, adjacency, pairInfo, config;
  private MemorySegment points,
      sources,
      measurements,
      members,
      modes,
      indices,
      ordering,
      depths,
      meta,
      values;
  private MemorySegment lookup, finish;
  private final long[] guarded = new long[7];
  private final long started = System.nanoTime();
  private double preparationSeconds;
  private long residentBytes;
  private long surfaceBudget;

  long surfaceBudget() {
    return surfaceBudget;
  }

  Object guards() {
    return Json.object(
        "threshold",
        guarded[0],
        "reprojection",
        guarded[1],
        "mode_tie",
        guarded[2],
        "independent_tie",
        guarded[3],
        "weighted_median",
        guarded[4],
        "lookup",
        guarded[5],
        "uncertainty",
        guarded[6]);
  }

  Object resources() {
    return Json.object(
        "device",
        cl.device.name(),
        "resident_evidence_bytes",
        residentBytes,
        "batch_capacity",
        batch,
        "preparation_seconds",
        preparationSeconds,
        "guard_reasons",
        guards(),
        "opencl_profile",
        cl.profile());
  }

  GpuFusion(
      Camera[] cameras,
      int[] widths,
      int[] heights,
      List<DepthFusion.Endpoint> evidence,
      int[][] pairImages,
      Options options,
      int requiredViews) {
    this(cameras, widths, heights, evidence, pairImages, options, requiredViews, 1L << 30);
  }

  GpuFusion(
      Camera[] cameras,
      int[] widths,
      int[] heights,
      List<DepthFusion.Endpoint> evidence,
      int[][] pairImages,
      Options options,
      int requiredViews,
      long bankCap) {
    cl = new GpuSgm.Compute(options, kernelSource(options));
    images = cameras.length;
    pairs = pairImages.length;
    endpoints = evidence.size();
    required = requiredViews;
    refine =
        options.depthRefinement ? options.depthRefinementMethod.equals("reprojection") ? 2 : 1 : 0;
    try {
      if (images == 0 || pairs == 0 || endpoints == 0)
        throw new IllegalArgumentException("No fusion evidence");
      long totalCells = 0;
      for (DepthFusion.Endpoint e : evidence) totalCells += e.map().length;
      long bankBytes = Math.min(bankCap, cl.device.allocation()) / 8 * 8;
      bankCells = Math.toIntExact(bankBytes / 8);
      long evidenceBytes = Math.multiplyExact(totalCells, 8L);
      residentBytes = evidenceBytes;
      if (bankBytes < 8 || evidenceBytes > bankBytes * 8)
        throw new IllegalArgumentException("Fusion evidence exceeds eight device allocation banks");
      long budget = GpuSgm.budget(cl.device, options);
      long fixed = evidenceBytes + 1048576L + images * 256L + endpoints * 32L;
      long perCandidate =
          24 + 4 + endpoints * 52L + images * 12L + pairs * 8L + (pairs + 1L) * 24 + 56;
      int capacity = options.fusionBatchSize;
      while (capacity >= 32
          && (fixed + capacity * perCandidate > budget
              || capacity * (long) endpoints * 48 > cl.device.allocation())) capacity /= 2;
      if (capacity < 32)
        throw new IllegalArgumentException(
            "Fusion depth evidence/scratch exceeds GPU memory budget (evidence="
                + Math.round(evidenceBytes / 1048576.)
                + " MiB, budget="
                + budget / 1048576
                + " MiB)");
      batch = capacity;
      surfaceBudget = Math.max(0, budget - fixed - capacity * perCandidate);
      System.out.printf(
          Locale.ROOT,
          "GPU fusion: %s; %d cameras, %d pairs; %.1f MiB evidence, batch %d%n",
          cl.device.name(),
          images,
          pairs,
          evidenceBytes / 1048576.,
          batch);
      for (int i = 0; i < 8; i++)
        banks[i] = cl.buffer(Math.max(8, Math.min(bankBytes, evidenceBytes - i * bankBytes)));
      double[] cdata = new double[images * 28];
      int[] size = new int[images * 2],
          info = new int[endpoints * 2],
          offsets = new int[images + 1],
          pi = new int[pairs * 2];
      long[] base = new long[endpoints];
      for (int i = 0; i < images; i++) {
        Camera c = cameras[i];
        int at = i * 28;
        System.arraycopy(c.rotation, 0, cdata, at, 9);
        System.arraycopy(c.translation, 0, cdata, at + 9, 3);
        cdata[at + 12] = c.fx;
        cdata[at + 13] = c.fy;
        cdata[at + 14] = c.cx;
        cdata[at + 15] = c.cy;
        System.arraycopy(c.center(), 0, cdata, at + 16, 3);
        System.arraycopy(Math3.transpose(c.rotation), 0, cdata, at + 19, 9);
        size[i * 2] = widths[i];
        size[i * 2 + 1] = heights[i];
      }
      long cursor = 0;
      long progress = System.nanoTime();
      try (Arena arena = Arena.ofConfined()) {
        MemorySegment staging = arena.allocate(16 * 1048576L, 8);
        for (int e = 0; e < endpoints; e++) {
          DepthFusion.Endpoint item = evidence.get(e);
          info[e * 2] = item.image();
          info[e * 2 + 1] = item.pair();
          base[e] = cursor;
          offsets[item.image() + 1]++;
          for (int at = 0; at < item.map().length; ) {
            int bank = (int) (cursor / bankCells), position = (int) (cursor % bankCells);
            int n =
                (int)
                    Math.min(
                        Math.min(staging.byteSize() / 8, item.map().length - at),
                        bankCells - position);
            for (int j = 0; j < n; j++) {
              staging.set(JAVA_INT, j * 8L, Float.floatToRawIntBits(item.map().get(at + j)));
              staging.set(JAVA_INT, j * 8L + 4, item.map().offset(at + j));
            }
            cl.transfer(banks[bank], position * 8L, staging.asSlice(0, n * 8L), false);
            at += n;
            cursor += n;
            if (System.nanoTime() - progress >= 5_000_000_000L) {
              System.out.printf(
                  Locale.ROOT,
                  "GPU fusion: uploaded %.1f / %.1f MiB depth evidence%n",
                  cursor * 8 / 1048576.,
                  evidenceBytes / 1048576.);
              progress = System.nanoTime();
            }
          }
        }
        for (int i = 1; i <= images; i++) offsets[i] += offsets[i - 1];
        for (int i = 0; i < pairs; i++) {
          pi[i * 2] = pairImages[i][0];
          pi[i * 2 + 1] = pairImages[i][1];
        }
        camera = upload(arena.allocateFrom(JAVA_DOUBLE, cdata));
        dimensions = upload(arena.allocateFrom(JAVA_INT, size));
        mapInfo = upload(arena.allocateFrom(JAVA_INT, info));
        starts = upload(arena.allocateFrom(JAVA_LONG, base));
        adjacency = upload(arena.allocateFrom(JAVA_INT, offsets));
        pairInfo = upload(arena.allocateFrom(JAVA_INT, pi));
        config =
            upload(
                arena.allocateFrom(
                    JAVA_DOUBLE,
                    options.fusionRelative,
                    options.fusionPixels,
                    options.fusionConsensus,
                    options.maxDepthUncertainty));
      }
      points = cl.buffer(batch * 24L);
      sources = cl.buffer(batch * 4L);
      measurements = cl.buffer(batch * (long) endpoints * 48);
      members = cl.buffer(batch * (long) endpoints * 4);
      modes = cl.buffer(batch * (long) images * 8);
      indices = cl.buffer(batch * (long) (pairs + images) * 4);
      ordering = cl.buffer(batch * (long) pairs * 4);
      depths = cl.buffer(batch * (pairs + 1L) * 24);
      meta = cl.buffer(batch * 16L);
      values = cl.buffer(batch * 40L);
      lookup = cl.kernel("lookup");
      finish = cl.kernel("finish_fusion");
      preparationSeconds = (System.nanoTime() - started) / 1e9;
      System.out.printf(
          Locale.ROOT, "GPU fusion: resident evidence ready in %.2f s%n", preparationSeconds);
    } catch (RuntimeException e) {
      cl.close();
      throw e;
    }
  }

  private MemorySegment upload(MemorySegment data) {
    MemorySegment b = cl.buffer(data.byteSize());
    cl.transfer(b, 0, data, false);
    return b;
  }

  DepthFusion.Result[] fuse(int[] source, double[][] world, int count) {
    return fuse(source, world, count, true);
  }

  DepthFusion.Result[] fuse(int[] source, double[][] world, int count, boolean refinement) {
    DepthFusion.Result[] result = new DepthFusion.Result[count];
    for (int start = 0; start < count; start += batch) {
      int n = Math.min(batch, count - start);
      try (Arena arena = Arena.ofConfined()) {
        MemorySegment hostPoints = arena.allocate(n * 24L),
            hostSources = arena.allocate(n * 4L),
            hostMeta = arena.allocate(n * 16L),
            hostValues = arena.allocate(n * 40L);
        for (int i = 0; i < n; i++) {
          for (int j = 0; j < 3; j++)
            hostPoints.set(JAVA_DOUBLE, i * 24L + j * 8L, world[start + i][j]);
          hostSources.set(JAVA_INT, i * 4L, source[start + i]);
        }
        cl.transfer(points, 0, hostPoints, false);
        cl.transfer(sources, 0, hostSources, false);
        // In-order queue: every work item owns its measurement; later kernels consume completed
        // arrays.
        cl.run(
            lookup,
            (long) n * endpoints,
            points,
            camera,
            dimensions,
            mapInfo,
            starts,
            config,
            banks[0],
            banks[1],
            banks[2],
            banks[3],
            banks[4],
            banks[5],
            banks[6],
            banks[7],
            measurements,
            members,
            n,
            endpoints,
            bankCells);
        cl.run(
            finish,
            n,
            points,
            sources,
            camera,
            mapInfo,
            adjacency,
            pairInfo,
            config,
            measurements,
            members,
            modes,
            indices,
            ordering,
            depths,
            meta,
            values,
            n,
            endpoints,
            images,
            pairs,
            required,
            refinement ? refine : 0);
        cl.transfer(meta, 0, hostMeta, true);
        cl.transfer(values, 0, hostValues, true);
        for (int i = 0; i < n; i++) {
          long m = i * 16L, v = i * 40L;
          int status = hostMeta.get(JAVA_INT, m), guard = hostMeta.get(JAVA_INT, m + 12);
          for (int bit = 0; bit < guarded.length; bit++)
            if ((guard & (1 << bit)) != 0) guarded[bit]++;
          if (guard != 0) continue; // Threshold/tie/non-finite ambiguity: exact CPU replay.
          if (status < 0 || status >= STATUS.length)
            throw new IllegalStateException("Invalid GPU fusion status " + status);
          result[start + i] =
              new DepthFusion.Result(
                  new double[] {
                    hostValues.get(JAVA_DOUBLE, v),
                    hostValues.get(JAVA_DOUBLE, v + 8),
                    hostValues.get(JAVA_DOUBLE, v + 16)
                  },
                  hostMeta.get(JAVA_INT, m + 4),
                  hostMeta.get(JAVA_INT, m + 8),
                  hostValues.get(JAVA_DOUBLE, v + 24),
                  hostValues.get(JAVA_DOUBLE, v + 32),
                  STATUS[status]);
        }
      }
    }
    return result;
  }

  public void close() {
    cl.close();
  }

  /**
   * Generate equivalent SoA accesses so adjacent GPU lanes read adjacent candidates for one map.
   */
  private static String kernelSource(Options o) {
    String source = KERNELS;
    if (o.fusionLayout.equals("coalesced")) {
      source =
          source
              .replace("size_t id=(size_t)candidate*E+e;", "int id=candidate;")
              .replace(
                  "__global double *m=out+id*6;",
                  "__global double *measure=out; __global double *m=out;");
      source = array(source, "member", index -> "MB(e)");
      source =
          array(
              source,
              "m",
              index -> {
                int at = index.lastIndexOf("*6");
                return at < 0
                    ? "MM(e," + index + ")"
                    : "MM("
                        + index.substring(0, at)
                        + ","
                        + (index.length() == at + 2 ? "0" : index.substring(at + 3))
                        + ")";
              });
      // Rewrite innermost index families before the arrays that reference them.
      for (String[] pair :
          new String[][] {
            {"order", "ORD"},
            {"ind", "IND"},
            {"by", "BY"},
            {"mb", "MB"},
            {"md", "MD"},
            {"dz", "DZ"},
            {"dw", "DW"},
            {"sorted", "SORT"}
          }) source = array(source, pair[0], index -> pair[1] + "(" + index + ")");
      source =
          """
#define MM(e,k) measure[(((size_t)(k)*E)+(e))*n+id]
#define MB(e) member[(size_t)(e)*n+id]
#define MD(c) modes[(size_t)(c)*n+id]
#define IND(p) indices[(size_t)(p)*n+id]
#define BY(c) indices[(size_t)(P+(c))*n+id]
#define ORD(i) ordering[(size_t)(i)*n+id]
#define DZ(i) scratch[(size_t)(i)*n+id]
#define DW(i) scratch[(size_t)(P+1+(i))*n+id]
#define SORT(i) scratch[(size_t)(2*(P+1)+(i))*n+id]
"""
              + source;
    }
    return (o.fusionWeightMath.equals("direct") ? "#define DIRECT_WEIGHT 1\n" : "")
        + (o.fusionMeasuredReprojection ? "#define MEASURED_REPROJECTION 1\n" : "")
        + source;
  }

  private static String array(
      String source, String name, java.util.function.Function<String, String> convert) {
    StringBuilder out = new StringBuilder();
    int cursor = 0;
    while (cursor < source.length()) {
      int start = source.indexOf(name + "[", cursor);
      if (start < 0) {
        out.append(source, cursor, source.length());
        break;
      }
      if (start > 0 && Character.isJavaIdentifierPart(source.charAt(start - 1))) {
        out.append(source, cursor, start + name.length());
        cursor = start + name.length();
        continue;
      }
      int end = start + name.length() + 1, depth = 1;
      for (; end < source.length() && depth > 0; end++) {
        char c = source.charAt(end);
        if (c == '[') depth++;
        else if (c == ']') depth--;
      }
      if (depth != 0) throw new IllegalStateException("Malformed internal kernel access");
      out.append(source, cursor, start)
          .append(convert.apply(source.substring(start + name.length() + 1, end - 1)));
      cursor = end;
    }
    return out.toString();
  }

  private static final String KERNELS =
      """
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#pragma OPENCL FP_CONTRACT OFF
// Scalar expressions intentionally preserve the Java operation order. No fast-math build flags.
double3 view(__global const double *c, int image, double3 p) {
  c += image*28;
  return (double3)((c[0]*p.x+c[1]*p.y+c[2]*p.z)+c[9],
    (c[3]*p.x+c[4]*p.y+c[5]*p.z)+c[10],(c[6]*p.x+c[7]*p.y+c[8]*p.z)+c[11]);
}
double3 unview(__global const double *c,int image,double3 p) {
  c += image*28; p -= (double3)(c[9],c[10],c[11]);
  return (double3)(c[19]*p.x+c[20]*p.y+c[21]*p.z,c[22]*p.x+c[23]*p.y+c[24]*p.z,c[25]*p.x+c[26]*p.y+c[27]*p.z);
}
double length3(double3 v) { return sqrt(v.x*v.x+v.y*v.y+v.z*v.z); }
int near(double a,double b) { return isfinite(a)&&isfinite(b)&&fabs(a-b)<=1e-8*fmax(1.0,fmax(fabs(a),fabs(b))); }
int compatible(double m,double z,double rel,int *guard) {
  if(m<=0 || z<=0) return 0;
  double a=fabs(m-z),b=rel*fmin(m,z); if(near(a,b))*guard|=1; return a<=b;
}
int reprojection(__global const double *c,int image,double3 a,double3 b,double tolerance,int *guard) {
  double3 x=view(c,image,a),y=view(c,image,b); c+=image*28;
  if(near(x.z,1e-8)||near(y.z,1e-8))*guard|=2;
  if(x.z<=1e-8||y.z<=1e-8)return 0;
  double dx=(c[12]*x.x/x.z+c[14])-(c[12]*y.x/y.z+c[14]);
  double dy=(c[13]*x.y/x.z+c[15])-(c[13]*y.y/y.z+c[15]);
  double d=hypot(dx,dy); if(near(d,tolerance))*guard|=2; return d<=tolerance;
}
int back(__global const double *c,int a,int b,double3 p,double3 q,double tolerance,int *guard) {
  return reprojection(c,a,p,q,tolerance,guard)&&reprojection(c,b,p,q,tolerance,guard);
}
double weight(__global const double *c,int a,int b,double3 p,double z) {
  __global const double *ca=c+a*28,*cb=c+b*28;
  double3 va=p-(double3)(ca[16],ca[17],ca[18]),vb=p-(double3)(cb[16],cb[17],cb[18]);
  double denominator=length3(va)*length3(vb);
  double3 cross=(double3)(va.y*vb.z-va.z*vb.y,va.z*vb.x-va.x*vb.z,va.x*vb.y-va.y*vb.x);
#ifdef DIRECT_WEIGHT
  double sine=denominator==0?0:fmin(1.0,length3(cross)/denominator);
#else
  double sine=denominator==0?0:sin((asin(fmin(1.0,length3(cross)/denominator))*57.29577951308232)*0.017453292519943295);
#endif
  double f=fmin(fmin(ca[12],ca[13]),fmin(cb[12],cb[13]));
  double sigma=z*.5/fmax(1e-9,f*sine);return 1/fmax(1e-20,sigma*sigma);
}
// Huber objective, first derivative, Gauss-Newton curvature and weighted squared residual.
double4 reproj_terms(__global const double *c,int image,int anchor,double3 center,double3 ray,
    double3 measured,double rho,double huber) {
  __global const double *k=c+image*28;
  double3 t=view(c,image,center);
  double3 d=(double3)((k[0]*ray.x+k[1]*ray.y)+k[2]*ray.z,
      (k[3]*ray.x+k[4]*ray.y)+k[5]*ray.z,(k[6]*ray.x+k[7]*ray.y)+k[8]*ray.z);
  double linkedZ=view(c,anchor,measured).z;
  double3 v=d+t*rho,q=d+t*(1/linkedZ);
  if(v.z<=1e-12||q.z<=1e-8)return (double4)(INFINITY,0,0,0);
  double du=(k[12]*v.x/v.z+k[14])-(k[12]*q.x/q.z+k[14]);
  double dv=(k[13]*v.y/v.z+k[15])-(k[13]*q.y/q.z+k[15]);
  double ju=k[12]*(t.x*v.z-v.x*t.z)/(v.z*v.z),jv=k[13]*(t.y*v.z-v.y*t.z)/(v.z*v.z);
  double r=hypot(du,dv),w=r<=huber?1:huber/r;
  return (double4)(r<=huber?.5*r*r:huber*(r-.5*huber),w*(ju*du+jv*dv),
      w*(ju*ju+jv*jv),w*r*r);
}
// Each map pixel stores the original float32 Z bits and signed packed offsets, never half precision.
uint2 pixel(ulong at,int bankCells,__global const uint2 *b0,__global const uint2 *b1,
 __global const uint2 *b2,__global const uint2 *b3,__global const uint2 *b4,__global const uint2 *b5,
 __global const uint2 *b6,__global const uint2 *b7) {
  int bank=(int)(at/(ulong)bankCells),i=(int)(at%(ulong)bankCells);
  switch(bank){case 0:return b0[i];case 1:return b1[i];case 2:return b2[i];case 3:return b3[i];
    case 4:return b4[i];case 5:return b5[i];case 6:return b6[i];default:return b7[i];}
}
__kernel void lookup(__global const double *points,__global const double *c,__global const int *sizes,
 __global const int *info,__global const ulong *base,__global const double *config,
 __global const uint2 *b0,__global const uint2 *b1,__global const uint2 *b2,__global const uint2 *b3,
 __global const uint2 *b4,__global const uint2 *b5,__global const uint2 *b6,__global const uint2 *b7,
 __global double *out,__global int *member,int n,int E,int bankCells) {
  size_t work=get_global_id(0);if(work>=(size_t)n*E)return;
  int e=work/n,candidate=work%n,image=info[e*2];size_t id=(size_t)candidate*E+e;
  __global double *m=out+id*6; m[0]=0;member[id]=0;
  double3 p=vload3(candidate,points),v=view(c,image,p);int guard=0;
  if(!isfinite(v.x)||!isfinite(v.y)||!isfinite(v.z)){member[id]=2;return;}
  if(near(v.z,0.0))guard=1;
  if(v.z<=0){member[id]=guard*2;return;}
  __global const double *camera=c+image*28;
  double px=camera[12]*v.x/v.z+camera[14],py=camera[13]*v.y/v.z+camera[15];
  int w=sizes[image*2],h=sizes[image*2+1];
  if(near(px,0)||near(px,w-1)||near(py,0)||near(py,h-1))guard=1;
  if(px<0||px>w-1||py<0||py>h-1){member[id]=guard*2;return;}
  int x=(int)floor(px+.5),y=(int)floor(py+.5);
  if(near(px+.5,floor(px+.5))||near(py+.5,floor(py+.5)))guard=1;
  double distance=INFINITY,second=INFINITY,u=0,t=0,z=0;
  for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
    int xx=x+dx,yy=y+dy;if(xx<0||xx>=w||yy<0||yy>=h)continue;
    uint2 value=pixel(base[e]+(ulong)yy*w+xx,bankCells,b0,b1,b2,b3,b4,b5,b6,b7);
    double depth=(double)as_float(value.x);if(depth<=0)continue;
    double sx=xx+(double)(short)(value.y&65535)/65534.0,sy=yy+(double)(short)(value.y>>16)/65534.0;
    double d=(sx-px)*(sx-px)+(sy-py)*(sy-py);
    if(d<distance){second=distance;distance=d;u=sx;t=sy;z=depth;}
    else if(d<second)second=d;
  }
  if(near(distance,second))guard=1;
  if(near(distance,config[1]*config[1]))guard=1;
  member[id]=guard*2;
  if(z==0||distance>config[1]*config[1])return;
  double3 world=unview(c,image,(double3)((u-camera[14])/camera[12]*z,(t-camera[15])/camera[13]*z,z));
  m[0]=z;m[1]=distance;m[2]=world.x;m[3]=world.y;m[4]=world.z;
  // Pair endpoint cameras are recovered by a separate tiny table in the following kernel.
  m[5]=0;
}
// A work item owns a complete candidate. Scratch arrays are bounded by batch size and pair count.
__kernel void finish_fusion(__global const double *points,__global const int *sources,
 __global const double *c,__global const int *info,__global const int *adj,__global const int *pi,
 __global const double *cfg,__global double *measure,__global int *member,__global double *modes,
 __global int *indices,__global int *ordering,__global double *scratch,__global int *meta,
 __global double *values,int n,int E,int C,int P,int required,int refine) {
  int id=get_global_id(0);if(id>=n)return;
  double3 p=vload3(id,points); int source=sources[id],a=pi[source*2],b=pi[source*2+1],guard=0;
  __global double *m=measure+(size_t)id*E*6,*md=modes+(size_t)id*C;
  __global int *mb=member+(size_t)id*E,*ind=indices+(size_t)id*(P+C),*by=ind+P,*order=ordering+(size_t)id*P;
  __global double *dz=scratch+(size_t)id*(P+1)*3,*dw=dz+P+1,*sorted=dw+P+1;
  __global int *out=meta+id*4;__global double *val=values+id*5;
  out[0]=1;out[1]=0;out[2]=0;out[3]=0;val[0]=p.x;val[1]=p.y;val[2]=p.z;val[3]=NAN;val[4]=0;
  for(int e=0;e<E;e++) {
    if(mb[e]&2)guard|=32;
    if(m[e*6]>0){int pair=info[e*2+1];m[e*6+5]=weight(c,pi[pair*2],pi[pair*2+1],(double3)(m[e*6+2],m[e*6+3],m[e*6+4]),m[e*6]);}
  }
  for(int image=0;image<C;image++) {
    int begin=adj[image],end=adj[image+1],best=-1,bestCount=0,total=0;double bestWeight=0;
    for(int e=begin;e<end;e++)if(m[e*6]>0)total++;
    for(int center=begin;center<end;center++)if(m[center*6]>0) {
      int count=0;double weights=0;
      for(int e=begin;e<end;e++)if(m[e*6]>0&&compatible(m[center*6],m[e*6],cfg[0],&guard)){count++;weights+=m[e*6+5];}
      if(best>=0&&count==bestCount&&near(weights,bestWeight)) {
        for(int e=begin;e<end;e++)if(m[e*6]>0&&
          compatible(m[center*6],m[e*6],cfg[0],&guard)!=compatible(m[best*6],m[e*6],cfg[0],&guard))guard|=4;
      }
      if(count>bestCount||(count==bestCount&&weights>bestWeight)){best=center;bestCount=count;bestWeight=weights;}
    }
    md[image]=0;
    if(best>=0&&bestCount>=(int)ceil(total*cfg[2])) {
      // Median is selected without changing the original membership iteration order.
      int rank=bestCount/2;
      for(int e=begin;e<end;e++)if(m[e*6]>0&&compatible(m[best*6],m[e*6],cfg[0],&guard)) {
        mb[e]|=1;int less=0,equalBefore=0;
        for(int j=begin;j<end;j++)if(m[j*6]>0&&compatible(m[best*6],m[j*6],cfg[0],&guard)) {
          if(m[j*6]<m[e*6])less++;else if(m[j*6]==m[e*6]&&j<e)equalBefore++;
        }
        if(less+equalBefore==rank)md[image]=m[e*6];
      }
    }
  }
  if(!compatible(md[a],view(c,a,p).z,cfg[0],&guard)||!compatible(md[b],view(c,b,p).z,cfg[0],&guard)){out[3]=guard;return;}
  for(int pair=0;pair<P;pair++)ind[pair]=-1;
  for(int image=0;image<C;image++)by[image]=-1;
  int views= a==b?1:2,independent=0;
  for(int image=0;image<C;image++) {
    double z=view(c,image,p).z;if(!compatible(md[image],z,cfg[0],&guard))continue;
    int supported=(image==a||image==b);
    for(int e=adj[image];e<adj[image+1];e++)if(mb[e]&1) {
      int pair=info[e*2+1];double3 q=(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
      if(pair==source||!compatible(m[e*6],z,cfg[0],&guard)||!back(c,a,b,p,q,cfg[1],&guard))continue;
      supported=1;int previous=ind[pair];
      if(previous<0){order[independent++]=pair;ind[pair]=e;}
      else {
        if(fabs(m[e*6+1]-m[previous*6+1])<=1e-12*fmax(1e-6,fmax(m[e*6+1],m[previous*6+1]))&&m[e*6+1]!=m[previous*6+1]&&
          length3((double3)(m[e*6+2]-m[previous*6+2],m[e*6+3]-m[previous*6+3],m[e*6+4]-m[previous*6+4]))>1e-9)guard|=8;
        if(m[e*6+1]<m[previous*6+1])ind[pair]=e;
      }
      if(by[image]<0||m[e*6+5]>m[by[image]*6+5])by[image]=e;
    }
    if(supported&&image!=a&&image!=b)views++;
  }
  out[1]=views;out[2]=independent;out[3]=guard;
  if(views<required){out[0]=2;return;}
  double anchorZ=view(c,a,p).z;int D=1;
  dz[0]=anchorZ;dw[0]=weight(c,a,b,p,anchorZ);
  for(int i=0;i<independent;i++) {
    int e=ind[order[i]];double3 q=(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);double z=view(c,a,q).z;
    if(z>0){int pair=order[i];dz[D]=z;dw[D]=weight(c,pi[pair*2],pi[pair*2+1],q,z);D++;}
  }
  for(int i=0;i<D;i++)sorted[i]=dw[i];
  for(int i=1;i<D;i++){double v=sorted[i];int j=i;while(j>0&&sorted[j-1]>v){sorted[j]=sorted[j-1];j--;}sorted[j]=v;}
  double medianWeight=sorted[D/2];
  for(int i=0;i<D;i++)dw[i]=fmin(dw[i],4*medianWeight);
  for(int i=1;i<D;i++){double z=dz[i],w=dw[i];int j=i;while(j>0&&dz[j-1]>z){dz[j]=dz[j-1];dw[j]=dw[j-1];j--;}dz[j]=z;dw[j]=w;}
  double total=0;for(int i=0;i<D;i++)total+=dw[i];
  double acc=0,medianDepth=anchorZ;for(int i=0;i<D;i++){acc+=dw[i];if(near(acc,total*.5)&&i+1<D&&fabs(dz[i+1]-dz[i])>1e-10)guard|=16;if(acc>=total*.5){medianDepth=dz[i];break;}}
  double sum=0,ws=0;for(int i=0;i<D;i++)if(compatible(dz[i],medianDepth,cfg[0],&guard)){sum+=dw[i]*dz[i];ws+=dw[i];}
  double refinedDepth=refine&&independent>0&&ws>0?sum/ws:anchorZ;
  double reprojSigma=0;
  if(refine==2&&independent>0) {
    __global const double *ca=c+a*28;
    double3 center=(double3)(ca[16],ca[17],ca[18]),ray=(p-center)*(1/anchorZ);
    double rho=1/anchorZ,trial=1/refinedDepth,huber=fmax(.25,cfg[1]);int observations=0;
    double4 first=(double4)(0),second=(double4)(0);
    for(int image=0;image<C;image++)if(image!=a&&(by[image]>=0||image==b)) {
      int e=by[image];double3 q=image==b?p:(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
      first+=reproj_terms(c,image,a,center,ray,q,rho,huber);
      second+=reproj_terms(c,image,a,center,ray,q,trial,huber);observations++;
    }
    if(trial>0&&second.x<first.x)rho=trial;
    for(int iteration=0;iteration<6;iteration++) {
      double4 f=(double4)(0);
      for(int image=0;image<C;image++)if(image!=a&&(by[image]>=0||image==b)) {
        int e=by[image];double3 q=image==b?p:(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
        f+=reproj_terms(c,image,a,center,ray,q,rho,huber);
      }
      if(!(f.z>1e-20)||!isfinite(f.y))break;
      double step=fmax(-rho*.25,fmin(rho*.25,-f.y/f.z));int improved=0;
      for(int line=0;line<8;line++) {
        trial=rho+step;double objective=0;
        for(int image=0;image<C;image++)if(image!=a&&(by[image]>=0||image==b)) {
          int e=by[image];double3 q=image==b?p:(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
          objective+=reproj_terms(c,image,a,center,ray,q,trial,huber).x;
        }
        if(trial>0&&objective<f.x){rho=trial;improved=1;break;}step*=.5;
      }
      if(!improved||fabs(step)<rho*1e-10)break;
    }
    double4 end=(double4)(0);
    for(int image=0;image<C;image++)if(image!=a&&(by[image]>=0||image==b)) {
      int e=by[image];double3 q=image==b?p:(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
      end+=reproj_terms(c,image,a,center,ray,q,rho,huber);
    }
    refinedDepth=1/rho;
    reprojSigma=fmax(.5,sqrt(end.w/max(1,2*observations-1)))/sqrt(fmax(1e-30,end.z))/rho;
  }
  double3 refined=unview(c,a,view(c,a,p)*(refinedDepth/anchorZ));
  double sigma=1/sqrt(fmax(1e-30,ws))/fmax(1e-12,refinedDepth)*sqrt(fmax(1.0,(double)D/max(1,views-1)));
  sigma=fmax(sigma,reprojSigma);
  val[3]=sigma;
  if(!back(c,a,b,p,refined,cfg[1],&guard)||!compatible(md[a],view(c,a,refined).z,cfg[0],&guard)||!compatible(md[b],view(c,b,refined).z,cfg[0],&guard)){out[0]=3;out[3]=guard;return;}
  int verified=a==b?1:2;
  for(int image=0;image<C;image++)if(by[image]>=0) {
    int e=by[image];double z=view(c,image,refined).z;double3 q=(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
    if(compatible(md[image],z,cfg[0],&guard)&&compatible(m[e*6],z,cfg[0],&guard)&&back(c,a,b,refined,q,cfg[1],&guard)
#ifdef MEASURED_REPROJECTION
      &&reprojection(c,image,refined,q,cfg[1],&guard)
#endif
      &&image!=a&&image!=b)verified++;
  }
  if(verified<required){out[0]=4;out[1]=verified;out[3]=guard;return;}
  sigma*=sqrt(fmax(1.0,(views-1.0)/max(1,verified-1)));val[3]=sigma;out[1]=verified;
  if(near(sigma,cfg[3]))guard|=64;
  if(sigma>cfg[3]){out[0]=5;out[3]=guard;return;}
  out[0]=0;val[0]=refined.x;val[1]=refined.y;val[2]=refined.z;val[4]=length3(refined-p);
  if(!isfinite(val[0])||!isfinite(val[1])||!isfinite(val[2])||!isfinite(sigma))guard|=1;
  out[3]=guard;
}
""";
}
