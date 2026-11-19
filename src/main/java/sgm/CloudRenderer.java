package sgm;

import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.util.*;
import java.util.stream.IntStream;

/**
 * Software point renderer of the 3D view (windowless, so SelfTest can check it). Every point is
 * projected (in parallel) into a per-pixel 64-bit value: depth bits high, point id low; the
 * nearest point wins by an atomic minimum, so the picture does not depend on thread order and the
 * buffer doubles as the pick index. Colours are resolved per pixel afterwards; eye-dome lighting
 * (shading by the depth of the neighbouring pixels) gives the cloud its relief without normals.
 */
final class CloudRenderer {
  private CloudRenderer() {}

  private static final VarHandle LONGS = MethodHandles.arrayElementVarHandle(long[].class);
  static final long EMPTY = Long.MAX_VALUE;
  /** Point ids: dense index, SPARSE | sparse index, or LIDAR | lidar reference index. */
  static final int SPARSE = 1 << 30, LIDAR = 1 << 29, OVERVIEW = 1 << 28;
  /** Colour of the lidar reference points. */
  static final int LIDAR_COLOUR = 0xff9800;

  enum Colour { RGB, HEIGHT, CLASS, ATTRIBUTE }

  /**
   * An orbit view: target (cloud coordinates), yaw (degrees, clockwise from north), pitch (degrees
   * below the horizon, 90 = straight down), distance, vertical field of view; orthographic for
   * a map-like top view.
   */
  record View(double[] target, double yaw, double pitch, double distance, double fov, boolean ortho, double roll) {
    View(double[] target, double yaw, double pitch, double distance, double fov, boolean ortho) {
      this(target,yaw,pitch,distance,fov,ortho,0);
    }
    View { target=target.clone(); }
    @Override public double[] target() { return target.clone(); }
    double[] forward() {
      double y = Math.toRadians(yaw), p = Math.toRadians(pitch);
      return new double[] {Math.cos(p) * Math.sin(y), Math.cos(p) * Math.cos(y), -Math.sin(p)};
    }

    double[] right() {
      double y = Math.toRadians(yaw);
      double p = Math.toRadians(pitch), a = Math.toRadians(roll);
      double[] r = {Math.cos(y), -Math.sin(y), 0};
      double[] u = {Math.sin(p)*Math.sin(y),Math.sin(p)*Math.cos(y),Math.cos(p)};
      return new double[] {r[0]*Math.cos(a)+u[0]*Math.sin(a),r[1]*Math.cos(a)+u[1]*Math.sin(a),u[2]*Math.sin(a)};
    }

    double[] up() {
      double[] f = forward(), r = right();
      return new double[] {r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]};
    }

    double[] eye() {
      double[] f = forward();
      return new double[] {target[0] - f[0] * distance, target[1] - f[1] * distance, target[2] - f[2] * distance};
    }

    View orbit(double dYaw, double dPitch) {
      return new View(target, ((yaw + dYaw) % 360 + 360) % 360, wrap(pitch + dPitch), distance, fov, ortho, roll);
    }

    View zoom(double factor) {
      return new View(target, yaw, pitch, Math.max(1e-3, distance * factor), fov, ortho, roll);
    }

    /** Moves the target by screen pixels (dx right, dy down) at the target's depth. */
    View pan(double dx, double dy, int height) {
      double perPixel = 2 * distance * Math.tan(Math.toRadians(fov) / 2) / Math.max(1, height);
      double[] r = right(), u = up(), t = target.clone();
      for (int k = 0; k < 3; k++) t[k] += (-dx * r[k] + dy * u[k]) * perPixel;
      return new View(t, yaw, pitch, distance, fov, ortho, roll);
    }

    View at(double[] t) {
      return new View(t.clone(), yaw, pitch, distance, fov, ortho, roll);
    }

    View withOrtho(boolean o) {
      return new View(target, yaw, pitch, distance, fov, o, roll);
    }

    static double wrap(double a) { return ((a % 360) + 360) % 360; }
    View withRoll(double r) { return new View(target,yaw,pitch,distance,fov,ortho,wrap(r)); }
    View shifted(double x,double y,double z) { return at(new double[] {target[0]+x,target[1]+y,target[2]+z}); }

    View withAngles(double y, double p) {
      return new View(target, wrap(y), wrap(p), distance, fov, ortho, 0);
    }
  }

  /** Screen projection of a view on a w x h image: {sx, sy, depth} or null when behind. */
  static final class Projector {
    final double ex, ey, ez, rx, ry, rz, ux, uy, uz, fx, fy, fz, focal, scale, cx, cy;
    final boolean ortho;

    Projector(View v, int w, int h) {
      double[] e = v.eye(), r = v.right(), u = v.up(), f = v.forward();
      ex = e[0]; ey = e[1]; ez = e[2];
      rx = r[0]; ry = r[1]; rz = r[2];
      ux = u[0]; uy = u[1]; uz = u[2];
      fx = f[0]; fy = f[1]; fz = f[2];
      double half = Math.tan(Math.toRadians(v.fov()) / 2);
      focal = h / 2.0 / half;
      scale = h / (2 * v.distance() * half);
      cx = w / 2.0;
      cy = h / 2.0;
      ortho = v.ortho();
    }

    boolean visible(CloudDisplay.Box b) {
      double[] c=b.centre();
      double radius=Math.sqrt(Math.pow(b.x1()-b.x0(),2)+Math.pow(b.y1()-b.y0(),2)+Math.pow(b.z1()-b.z0(),2))/2;
      double dx=c[0]-ex,dy=c[1]-ey,dz=c[2]-ez,depth=dx*fx+dy*fy+dz*fz;
      if(depth+radius<=1e-6) return false;
      double a=dx*rx+dy*ry+dz*rz,u=dx*ux+dy*uy+dz*uz;
      if(ortho) return Math.abs(a)<=cx/scale+radius && Math.abs(u)<=cy/scale+radius;
      double tx=cx/focal,ty=cy/focal;
      return Math.abs(a)<=depth*tx+radius*Math.sqrt(1+tx*tx)
          && Math.abs(u)<=depth*ty+radius*Math.sqrt(1+ty*ty);
    }
    double pixelSpan(CloudDisplay.Box b) {
      double[] c=b.centre();
      double radius=Math.sqrt(Math.pow(b.x1()-b.x0(),2)+Math.pow(b.y1()-b.y0(),2)+Math.pow(b.z1()-b.z0(),2))/2;
      double depth=(c[0]-ex)*fx+(c[1]-ey)*fy+(c[2]-ez)*fz;
      return 2*radius*(ortho?scale:focal/Math.max(1e-6,depth-radius));
    }
    double[] unproject(double x,double y,double depth) {
      double a=(x-cx)/(ortho?scale:focal/depth),b=(cy-y)/(ortho?scale:focal/depth);
      return new double[] {ex+fx*depth+rx*a+ux*b,ey+fy*depth+ry*a+uy*b,ez+fz*depth+rz*a+uz*b};
    }

    /** Fills out {sx, sy, depth}; false when the point is behind the eye. */
    boolean project(double x, double y, double z, double[] out) {
      double dx = x - ex, dy = y - ey, dz = z - ez;
      double depth = dx * fx + dy * fy + dz * fz;
      if (!(depth > 1e-6)) return false;
      double a = dx * rx + dy * ry + dz * rz, b = dx * ux + dy * uy + dz * uz;
      if (ortho) {
        out[0] = cx + a * scale;
        out[1] = cy - b * scale;
      } else {
        out[0] = cx + focal * a / depth;
        out[1] = cy - focal * b / depth;
      }
      out[2] = depth;
      return true;
    }
  }

  /** What to draw. step > 1 draws every step-th dense point (the fast preview while moving). */
  record Request(CloudData data, View view, int width, int height, Colour colour, int attribute, int pointSize, boolean edl,
                 boolean dense, boolean sparse, int step, float lowValue, float highValue, CloudDisplay display,
                 java.util.function.BooleanSupplier cancelled, int pointBudget, CloudData overview) {
    Request(CloudData d,View v,int w,int h,Colour c,int a,int size,boolean edl,boolean dense,boolean sparse,int step,float lo,float hi,CloudDisplay display,java.util.function.BooleanSupplier cancelled,int budget) {
      this(d,v,w,h,c,a,size,edl,dense,sparse,step,lo,hi,display,cancelled,budget,null);
    }
    Request(CloudData d,View v,int w,int h,Colour c,int a,int size,boolean edl,boolean dense,boolean sparse,int step,float lo,float hi) {
      this(d,v,w,h,c,a,size,edl,dense,sparse,step,lo,hi,CloudDisplay.DEFAULT,()->false,Integer.MAX_VALUE);
    }
  }

  /** The picture and its per-pixel nearest-point buffer (for picking). */
  record Frame(int width, int height, int[] argb, long[] buffer, Request request) {
    /** Nearest point id within radius pixels of (px, py), or -1. */
    int pick(int px, int py, int radius) {
      long best = EMPTY; int distance=Integer.MAX_VALUE;
      for (int y = Math.max(0, py - radius); y <= Math.min(height - 1, py + radius); y++)
        for (int x = Math.max(0, px - radius); x <= Math.min(width - 1, px + radius); x++) {
          long v=buffer[y*width+x];int ds=(x-px)*(x-px)+(y-py)*(y-py);
          if(v!=EMPTY && ds<=radius*radius && (ds<distance || ds==distance && v<best)) {best=v;distance=ds;}
        }
      return best == EMPTY ? -1 : (int) (best & 0xffffffffL);
    }
  }

  static Frame render(Request q) { return render(q,null,null); }
  static Frame render(Request q,long[] reuse,int[] pixels) {return render(q,reuse,pixels,null);}
  static Frame render(Request q,long[] reuse,int[] pixels,float[] shadeReuse) {
    if(q.cancelled().getAsBoolean()) throw new java.util.concurrent.CancellationException();
    int w = q.width(), h = q.height();
    long[] buffer = reuse!=null && reuse.length==w*h?reuse:new long[w*h];
    Arrays.fill(buffer, EMPTY);
    CloudData d = q.data();
    Projector pr = new Projector(q.view(), w, h);
    int size = Math.max(1, q.pointSize());
    int contextBudget=q.overview()==null?0:d.n==0?q.pointBudget():Math.max(1,q.pointBudget()/2);
    if(q.dense() && q.overview()!=null)
      projectDense(q,q.overview(),pr,buffer,size,OVERVIEW,contextBudget);
    if(q.dense() && d.n>0)
      projectDense(q,d,pr,buffer,size,0,q.pointBudget()==Integer.MAX_VALUE?Integer.MAX_VALUE:Math.max(1,q.pointBudget()-contextBudget));
    if (d.showLidar && d.lx.length > 0) {
      int step = Math.max(Math.max(1,q.step()),(int)Math.max(1,Math.ceil(d.lx.length/(double)Math.max(1,q.pointBudget()))));
      double[] s = new double[3];
      for (int i = 0; i < d.lx.length; i += step) if (q.display().position(d.lx[i],d.ly[i],d.lz[i]) && pr.project(d.lx[i], d.ly[i], d.lz[i], s)) splat(buffer, w, h, s, size, LIDAR | i);
    }
    if (q.sparse() && d.sx.length > 0) {
      double[] s = new double[3];
      int sparseSize = Math.max(2, size + 1);
      boolean[] hidden = d.shidden, big = d.sbig;
      for (int i = 0; i < d.sx.length; i++) {
        if((i&4095)==0 && q.cancelled().getAsBoolean())throw new java.util.concurrent.CancellationException();
        if (hidden != null && hidden[i]) continue;
        if (q.display().position(d.sx[i],d.sy[i],d.sz[i]) && pr.project(d.sx[i], d.sy[i], d.sz[i], s)) splat(buffer, w, h, s, big != null && big[i] ? sparseSize + 1 : sparseSize, SPARSE | i);
      }
    }
    if(q.cancelled().getAsBoolean()) throw new java.util.concurrent.CancellationException();
    int[] argb = pixels!=null && pixels.length==w*h?pixels:new int[w*h];
    float[] attr = q.colour() == Colour.ATTRIBUTE && q.attribute() >= 0 && q.attribute() < d.attributes.size() ? d.attributes.get(q.attribute()).values() : null;
    float lo = q.lowValue(), hi = q.highValue(), range = hi > lo ? hi - lo : 1;
    int sparseMax = 2;
    for (int o : d.sobs) sparseMax = Math.max(sparseMax, Math.min(20, o));
    final int sm = sparseMax;
    CloudData.Attribute contextAttribute=q.overview()!=null && q.colour()==Colour.ATTRIBUTE
        && q.attribute()>=0 && q.attribute()<d.attributes.size()
        ?q.overview().attribute(d.attributes.get(q.attribute()).name()):null;
    final float[] contextValues=contextAttribute==null?null:contextAttribute.values();
    float[] logDepth=q.edl()?(shadeReuse!=null && shadeReuse.length==w*h?shadeReuse:new float[w*h]):null;
    if(logDepth!=null)IntStream.range(0,h).parallel().forEach(y->{
      if(q.cancelled().getAsBoolean())return;
      for(int x=0;x<w;x++){int i=y*w+x;logDepth[i]=buffer[i]==EMPTY?Float.NaN:(float)(Math.log(depth(buffer[i]))/Math.log(2));}
    });
    IntStream.range(0, h).parallel().forEach(y -> {
      if(q.cancelled().getAsBoolean()) return;
      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        long v = buffer[p];
        if (v == EMPTY) {
          argb[p] = background(y, h);
          continue;
        }
        int id = (int) (v & 0xffffffffL);
        int c;
        if ((id & SPARSE) != 0) {
          int i = id & ~SPARSE;
          // Tie points: bright end of the ramp (blue few images, red many), not shaded.
          int[] own = d.scolour;
          c = own != null ? own[i] & 0xffffff : ramp(.3f + .7f * Math.min(1f, (Math.min(20, d.sobs[i]) - 2f) / Math.max(1, sm - 2)));
          argb[p] = 0xff000000 | c;
          continue;
        } else if ((id & OVERVIEW) != 0) {
          int i=id&~OVERVIEW;CloudData coarse=q.overview();float value=contextValues==null?Float.NaN:contextValues[i];
          c=q.display().denseColour()!=null?q.display().denseColour():switch(q.colour()){
            case RGB->coarse.rgb[i];case HEIGHT->ramp(normalize(coarse.z[i],lo,hi));case CLASS->classColour(coarse.cls[i]);case ATTRIBUTE->Float.isNaN(value)?0x808080:ramp(normalize(value,lo,hi));
          };
        } else if ((id & LIDAR) != 0) c = q.display().lidarColour();
        else
          c = q.display().denseColour()!=null?q.display().denseColour():switch (q.colour()) {
            case RGB -> d.rgb[id];
            case HEIGHT -> ramp(normalize(d.z[id],lo,hi));
            case CLASS -> classColour(d.cls[id]);
            case ATTRIBUTE -> attr == null || Float.isNaN(attr[id]) ? 0x808080 : ramp(normalize(attr[id],lo,hi));
          };
        if (q.edl()) c = shade(c, edl(logDepth, w, h, x, y), q.display().shading());
        argb[p] = 0xff000000 | c;
      }
    });
    if(q.cancelled().getAsBoolean()) throw new java.util.concurrent.CancellationException();
    return new Frame(w, h, argb, buffer, q);
  }

  /** Both the resident overview and local detail use spatial culling and bounded parallel work. */
  private static void projectDense(Request q,CloudData d,Projector pr,long[] buffer,int size,int marker,int budget) {
    int[] selected=d.spatial==null?null:d.spatial.select(pr,q.display().box(),
        Math.min(budget,Math.max(1,d.n/Math.max(1,q.step()))),q.cancelled());
    int step=Math.max(Math.max(1,q.step()),(int)Math.max(1,Math.ceil(d.n/(double)Math.max(1,budget))));
    int count=selected==null?(d.n+step-1)/step:selected.length;
    int chunks=Math.max(1,Math.min(256,count/50_000+1));
    IntStream.range(0,chunks).parallel().forEach(chunk->{
      double[] screen=new double[3];
      for(int k=(int)((long)count*chunk/chunks);k<(long)count*(chunk+1)/chunks;k++) {
        if((k&4095)==0 && q.cancelled().getAsBoolean())return;
        int i=selected==null?k*step:selected[k];
        if(q.display().dense(d,i) && pr.project(d.x[i],d.y[i],d.z[i],screen))
          splat(buffer,q.width(),q.height(),screen,size,marker|i);
      }
    });
  }

  private static void splat(long[] buffer, int w, int h, double[] s, int size, int id) {
    int x0 = (int) Math.floor(s[0] - (size - 1) / 2.0), y0 = (int) Math.floor(s[1] - (size - 1) / 2.0);
    if (x0 >= w || y0 >= h || x0 + size <= 0 || y0 + size <= 0) return;
    long packed = (long) Float.floatToIntBits((float) s[2]) << 32 | (id & 0xffffffffL);
    for (int y = Math.max(0, y0); y < Math.min(h, y0 + size); y++)
      for (int x = Math.max(0, x0); x < Math.min(w, x0 + size); x++) {
        int p = y * w + x;
        long old = (long) LONGS.getVolatile(buffer, p);
        while (packed < old && !LONGS.compareAndSet(buffer, p, old, packed)) old = (long) LONGS.getVolatile(buffer, p);
      }
  }

  static float depth(long v) {
    return Float.intBitsToFloat((int) (v >>> 32));
  }

  static final int[] EDL_X={-2,2,0,0,-1,1,-1,1},EDL_Y={0,0,-2,2,-1,-1,1,1};
  static float edl(float[] log,int w,int h,int x,int y) {
    float centre=log[y*w+x],sum=0;
    for(int i=0;i<8;i++){int nx=x+EDL_X[i],ny=y+EDL_Y[i];if(nx<0||ny<0||nx>=w||ny>=h)continue;float d=centre-log[ny*w+nx];if(d>0)sum+=d;}
    return sum;
  }

  /** Eye-dome lighting: how much nearer the neighbouring points are (log depth), 0 = none. */
  static float edl(long[] buffer, int w, int h, int x, int y, long v) {
    double centre = Math.log(depth(v)) / Math.log(2), sum = 0;
    for (int i=0;i<8;i++) {
      int nx = x + EDL_X[i], ny = y + EDL_Y[i];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      long n = buffer[ny * w + nx];
      if (n == EMPTY) continue;
      double d = centre - Math.log(depth(n)) / Math.log(2);
      if (d > 0) sum += d;
    }
    return (float) sum;
  }

  static int shade(int rgb, float response) {
    return shade(rgb,response,12);
  }
  static int shade(int rgb,float response,double strength) {
    double k = Math.exp(-strength * response);
    int r = (int) ((rgb >> 16 & 255) * k), g = (int) ((rgb >> 8 & 255) * k), b = (int) ((rgb & 255) * k);
    return r << 16 | g << 8 | b;
  }

  static int background(int y, int h) {
    int top = 0x2a, bottom = 0x14, v = top + (bottom - top) * y / Math.max(1, h - 1);
    return 0xff000000 | v << 16 | v << 8 | (v + 6);
  }

  static float normalize(float value,float lo,float hi) {return hi>lo?(value-lo)/(hi-lo):.5f;}

  /** Turbo-like colour ramp over 0..1. */
  static int ramp(float t) {
    double x = Math.max(0, Math.min(1, Float.isNaN(t) ? 0 : t));
    double r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
    double g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
    double b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
    return clamp(r) << 16 | clamp(g) << 8 | clamp(b);
  }

  private static int clamp(double v) {
    return (int) Math.max(0, Math.min(255, Math.round(v * 255)));
  }

  /** ASPRS classes: ground brown, vegetation greens, building orange, noise magenta, water blue. */
  static int classColour(int c) {
    return switch (c & 255) {
      case 2 -> 0xa0703c;
      case 3 -> 0x9fd36b;
      case 4 -> 0x4caf50;
      case 5 -> 0x1b7a2e;
      case 6 -> 0xf08c2a;
      case 7, 18 -> 0xe040fb;
      case 9 -> 0x3b82f6;
      default -> 0xb8b8b8;
    };
  }

  /** The frustum lines of a camera: apex then four corners at size s (cloud coordinates). */
  static double[][] frustum(CloudData.Cam c, double s) {
    double hw = c.width() / 2.0 / c.fx() * s, hh = c.height() / 2.0 / c.fx() * s;
    double[][] local = {{0, 0, 0}, {-hw, -hh, s}, {hw, -hh, s}, {hw, hh, s}, {-hw, hh, s}};
    double[][] out = new double[5][];
    double[] r = c.r();
    for (int i = 0; i < 5; i++) {
      double[] p = local[i];
      // world = R^T p + C
      out[i] = new double[] {
        r[0] * p[0] + r[3] * p[1] + r[6] * p[2] + c.center()[0],
        r[1] * p[0] + r[4] * p[1] + r[7] * p[2] + c.center()[1],
        r[2] * p[0] + r[5] * p[1] + r[8] * p[2] + c.center()[2]};
    }
    return out;
  }

  /** A view that shows everything: looking down at 55 degrees from the south, all points inside. */
  static View home(CloudData d) {
    // The points' 1%..99% range (stray tie points far outside the block would shrink the scene to
    // a dot), then the cameras near it (a camera thrown far away by a failed alignment too).
    double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    boolean any = false;
    float[][] axes = d.n > 0 ? new float[][] {d.x, d.y, d.z} : new float[][] {d.sx, d.sy, d.sz};
    int count = d.n > 0 ? d.n : d.sx.length;
    if(d.detailRegion!=null && d.sourceIndex!=null && d.sourceIndex.root!=null) {
      var b=d.sourceIndex.root.bounds;
      lo=new double[]{b.x0()-d.origin[0],b.y0()-d.origin[1],b.z0()-d.origin[2]};
      hi=new double[]{b.x1()-d.origin[0],b.y1()-d.origin[1],b.z1()-d.origin[2]};any=true;
    } else if (count > 0) {
      any = true;
      for (int k = 0; k < 3; k++) {
        lo[k] = CloudData.quantile(axes[k], count, .01);
        hi[k] = CloudData.quantile(axes[k], count, .99);
      }
    }
    if (!d.cameras.isEmpty()) {
      double[] c0 = any ? new double[] {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2} : null;
      double reach = any ? 3 * Math.max(1, Math.sqrt(Math.pow(hi[0] - lo[0], 2) + Math.pow(hi[1] - lo[1], 2) + Math.pow(hi[2] - lo[2], 2))) : Double.MAX_VALUE;
      for (CloudData.Cam c : d.cameras) {
        double[] p = c.center();
        if (c0 != null && Math.sqrt(Math.pow(p[0] - c0[0], 2) + Math.pow(p[1] - c0[1], 2) + Math.pow(p[2] - c0[2], 2)) > reach) continue;
        any = true;
        grow(lo, hi, p[0], p[1], p[2]);
      }
    }
    if (!any) return new View(new double[3], 0, 55, 100, 45, false);
    double[] t = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
    double radius = Math.max(1, Math.sqrt(Math.pow(hi[0] - lo[0], 2) + Math.pow(hi[1] - lo[1], 2) + Math.pow(hi[2] - lo[2], 2)) / 2);
    return new View(t, 0, 55, radius / Math.tan(Math.toRadians(22.5)) * 1.05, 45, false);
  }

  private static void grow(double[] lo, double[] hi, double x, double y, double z) {
    lo[0] = Math.min(lo[0], x); lo[1] = Math.min(lo[1], y); lo[2] = Math.min(lo[2], z);
    hi[0] = Math.max(hi[0], x); hi[1] = Math.max(hi[1], y); hi[2] = Math.max(hi[2], z);
  }

  /** A sensible frustum size: a third of the median distance to the nearest other camera. */
  static double cameraSize(CloudData d) {
    List<CloudData.Cam> c = d.cameras;
    if (c.size() < 2) return 5;
    double[] nearest = new double[c.size()];
    for (int i = 0; i < c.size(); i++) {
      double best = Double.MAX_VALUE;
      for (int j = 0; j < c.size(); j++)
        if (i != j) best = Math.min(best, Math.sqrt(Math.pow(c.get(i).center()[0] - c.get(j).center()[0], 2) + Math.pow(c.get(i).center()[1] - c.get(j).center()[1], 2) + Math.pow(c.get(i).center()[2] - c.get(j).center()[2], 2)));
      nearest[i] = best;
    }
    Arrays.sort(nearest);
    return Math.max(1e-3, nearest[nearest.length / 2] / 3);
  }
}
