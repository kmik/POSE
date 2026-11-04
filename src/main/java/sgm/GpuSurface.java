package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.*;

/**
 * Bounded FP64 neighborhood predicates. CPU owns ordering, resolved layers and representative
 * choices.
 */
final class GpuSurface implements AutoCloseable {
  static final int BATCH = 65536;
  private final GpuSgm.Compute cl;
  private final MemorySegment points, pairs, cameras, queries, result, kernel, config;
  long tested, guarded, audited;

  GpuSurface(Camera[] camera, int[][] endpoints, int capacity, Options o, long available) {
    long bytes = (long) capacity * 64 + BATCH * 12L + camera.length * 128L + endpoints.length * 8L;
    if (bytes
        > Math.min(
            available, Math.min(128L * 1048576, GpuSgm.budget(GpuSgm.selectedDevice(o), o) / 16)))
      throw new IllegalArgumentException(
          "Surface neighborhood device working set exceeds reserved budget");
    cl = new GpuSgm.Compute(o, SOURCE);
    try (Arena arena = Arena.ofConfined()) {
      MemorySegment data = arena.allocate(camera.length * 128L);
      for (int i = 0; i < camera.length; i++) {
        int k = 0;
        for (double v : camera[i].rotation) data.set(JAVA_DOUBLE, i * 128L + 8L * k++, v);
        for (double v : camera[i].translation) data.set(JAVA_DOUBLE, i * 128L + 8L * k++, v);
        for (double v : new double[] {camera[i].fx, camera[i].fy, camera[i].cx, camera[i].cy})
          data.set(JAVA_DOUBLE, i * 128L + 8L * k++, v);
      }
      cameras = cl.buffer(data.byteSize());
      cl.transfer(cameras, 0, data, false);
      data = arena.allocate(endpoints.length * 8L);
      for (int i = 0; i < endpoints.length; i++) {
        data.set(JAVA_INT, i * 8L, endpoints[i][0]);
        data.set(JAVA_INT, i * 8L + 4, endpoints[i][1]);
      }
      pairs = cl.buffer(data.byteSize());
      cl.transfer(pairs, 0, data, false);
      data =
          arena.allocateFrom(
              JAVA_DOUBLE,
              o.surfaceRadius * o.step,
              o.surfaceMaxShift,
              Math.cos(Math.toRadians(o.surfaceNormalAngle)),
              o.fusionPixels);
      config = cl.buffer(data.byteSize());
      cl.transfer(config, 0, data, false);
      points = cl.buffer(capacity * 64L);
      queries = cl.buffer(BATCH * 8L);
      result = cl.buffer(BATCH * 4L);
      kernel = cl.kernel("neighborhood");
    } catch (RuntimeException e) {
      cl.close();
      throw e;
    }
  }

  void partition(List<SurfaceConsolidator.Sample> samples) {
    try (Arena a = Arena.ofConfined()) {
      MemorySegment data = a.allocate(samples.size() * 64L);
      for (int i = 0; i < samples.size(); i++) {
        var p = samples.get(i);
        long at = i * 64L;
        for (int k = 0; k < 3; k++) {
          data.set(JAVA_DOUBLE, at + k * 8L, p.point[k]);
          data.set(JAVA_DOUBLE, at + (k + 3) * 8L, p.normal == null ? Double.NaN : p.normal[k]);
        }
        data.set(JAVA_DOUBLE, at + 48, p.footprint);
        data.set(JAVA_DOUBLE, at + 56, p.source);
      }
      cl.transfer(points, 0, data, false);
    }
  }

  int[] filter(
      int[] anchor, int[] neighbor, int count, double radius, double height, double cosine) {
    try (Arena a = Arena.ofConfined()) {
      MemorySegment q = a.allocate(count * 8L), out = a.allocate(count * 4L);
      for (int i = 0; i < count; i++) {
        q.set(JAVA_INT, i * 8L, anchor[i]);
        q.set(JAVA_INT, i * 8L + 4, neighbor[i]);
      }
      cl.transfer(queries, 0, q, false);
      cl.run(kernel, count, points, pairs, cameras, queries, result, config, count);
      cl.transfer(result, 0, out, true);
      int[] flags = new int[count];
      for (int i = 0; i < count; i++) {
        flags[i] = out.get(JAVA_INT, i * 4L);
        if (flags[i] == 2) guarded++;
      }
      tested += count;
      return flags;
    }
  }

  Object report() {
    return Json.object(
        "tested_neighbors",
        tested,
        "cpu_guarded_neighbors",
        guarded,
        "cpu_audited_neighbors",
        audited,
        "opencl_profile",
        cl.profile());
  }

  public void close() {
    if (!telemetryClosed) {
      RunTelemetry.gpuProfile("Surface geometry", cl.profile());
      telemetryClosed = true;
    }
    cl.close();
  }

  private boolean telemetryClosed;

  private static final String SOURCE =
      """
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#pragma OPENCL FP_CONTRACT OFF
double3 view(__global const double *c,int id,double3 p){c+=id*16;
 return (double3)((c[0]*p.x+c[1]*p.y+c[2]*p.z)+c[9],(c[3]*p.x+c[4]*p.y+c[5]*p.z)+c[10],(c[6]*p.x+c[7]*p.y+c[8]*p.z)+c[11]);}
int near(double a,double b){return isfinite(a)&&isfinite(b)&&fabs(a-b)<=1e-8*fmax(1.,fmax(fabs(a),fabs(b)));}
int project(__global const double *c,int id,double3 p,double3 q,double tolerance,int *guard){
 double3 x=view(c,id,p),y=view(c,id,q);c+=id*16;
 if(near(x.z,1e-8)||near(y.z,1e-8))*guard=1;
 if(x.z<=1e-8||y.z<=1e-8)return 0;
 double dx=(c[12]*x.x/x.z+c[14])-(c[12]*y.x/y.z+c[14]);
 double dy=(c[13]*x.y/x.z+c[15])-(c[13]*y.y/y.z+c[15]);
 double d=hypot(dx,dy);if(near(d,tolerance))*guard=1;return d<=tolerance;
}
int back(__global const double *c,__global const int *pairs,int source,double3 p,double3 q,double tolerance,int *guard){
 return project(c,pairs[source*2],p,q,tolerance,guard)&&project(c,pairs[source*2+1],p,q,tolerance,guard);
}
__kernel void neighborhood(__global const double *data,__global const int *pairs,__global const double *c,
 __global const int *queries,__global int *out,__global const double *config,int count){
 int i=get_global_id(0);if(i>=count)return;
 double radiusScale=config[0],heightScale=config[1],cosine=config[2],pixels=config[3];
 __global const double *a=data+queries[i*2]*8,*b=data+queries[i*2+1]*8;
 double3 p=(double3)(a[0],a[1],a[2]),q=(double3)(b[0],b[1],b[2]);
 double nx=a[3],ny=a[4],nz=a[5];int guard=0,ok=0;
 if(isfinite(b[3])){
  double normal=nx*b[3]+ny*b[4]+nz*b[5];if(near(normal,cosine))guard=1;
  if(normal>=cosine){
   double dx=q.x-p.x,dy=q.y-p.y,dz=q.z-p.z;
   double offset=dx*nx+dy*ny+dz*nz,tangent=fmax(0.,dx*dx+dy*dy+dz*dz-offset*offset);
   double radius=radiusScale*a[6],height=heightScale*a[6];
   if(near(fabs(offset),height)||near(tangent,radius*radius))guard=1;
   if(fabs(offset)<=height&&tangent<=radius*radius){
    double3 pa=(double3)(p.x+nx*offset,p.y+ny*offset,p.z+nz*offset);
    double3 qb=(double3)(q.x+nx*(-offset),q.y+ny*(-offset),q.z+nz*(-offset));
    ok=back(c,pairs,(int)a[7],p,pa,pixels,&guard)&&back(c,pairs,(int)b[7],q,qb,pixels,&guard);
   }
  }
 }
 out[i]=guard?2:ok;
}
""";
}
