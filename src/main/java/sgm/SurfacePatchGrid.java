package sgm;

import java.util.*;
import java.util.function.IntToDoubleFunction;

/** Bounded multiscale, multilayer inverse-depth context. Models predict only inside
 * their measured convex hull; separately fitted layers are never averaged.
 * No point is moved. One cache of immutable models belongs to one evidence map. */
final class SurfacePatchGrid {
  record Prediction(int kind,double depth,double sigma) {
    static final Prediction UNKNOWN=new Prediction(0,Double.NaN,Double.NaN);
  }
  private record Plane(double a,double b,double c,double sigma,double[] hull,int scale) {
    double at(double x,double y){return a*x+b*y+c;}
  }
  private static final int CELL=16;
  private final int width,height,columns;
  private final double relative;
  private final IntToDoubleFunction depth;
  // Lazy bounded tiles: no full-raster copy, thread-safe publication of immutable models.
  private record Entry(int key,Plane[] planes) {}
  private final java.util.concurrent.atomic.AtomicReferenceArray<Entry> models;

  SurfacePatchGrid(int width,int height,IntToDoubleFunction depth,double relative) {
    this(width,height,depth,relative,2048);
  }
  SurfacePatchGrid(int width,int height,IntToDoubleFunction depth,double relative,int cacheCells) {
    this.width=width;this.height=height;this.depth=depth;this.relative=relative;
    columns=Math.ceilDiv(width,CELL);
    // Models are deterministic, so cache size never changes results. Covering the whole grid
    // (when the memory budget allows) avoids rebuilding evicted cells.
    int cells=columns*Math.ceilDiv(height,CELL);
    models=new java.util.concurrent.atomic.AtomicReferenceArray<>(Math.max(16,Math.min(cells,cacheCells)));
  }
  static int cacheCells(Options o,int rasters) {
    long budget=Math.min(o.memoryMb*1048576L,Runtime.getRuntime().maxMemory());
    return (int)Math.max(16,Math.min(1<<16,budget/16/Math.max(1,rasters)/4096));
  }

  /** 0 unknown/protected, 1 compatible surface, 2 in front of model, 3 behind model. */
  Prediction predict(double x,double y,double z) {
    if(!(z>0)||x<0||y<0||x>=width||y>=height)return Prediction.UNKNOWN;
    int cx=(int)x/CELL,cy=(int)y/CELL,index=cy*columns+cx;
    int slot=Math.floorMod(index,models.length());Entry cached=models.get(slot);
    Plane[] planes=cached!=null&&cached.key==index?cached.planes:null;
    if(planes==null){planes=build(cx*CELL+CELL/2,cy*CELL+CELL/2);models.set(slot,new Entry(index,planes));}
    int scale=0;
    for(Plane p:planes)if(inside(p.hull,x,y))scale=Math.max(scale,p.scale);
    if(scale==0)return Prediction.UNKNOWN;
    Plane closest=null;double distance=Double.POSITIVE_INFINITY;int direction=0;
    for(Plane p:planes)if(p.scale==scale&&inside(p.hull,x,y)) {
      double inverse=p.at(x,y);if(!(inverse>0))continue;
      double predicted=1/inverse,sigma=p.sigma/(inverse*inverse);
      double band=Math.max(relative*Math.min(z,predicted),3*sigma);
      if(Math.abs(z-predicted)<=band)return new Prediction(1,predicted,sigma);
      int side=z<predicted?2:3;
      // Competing foreground/background layers are ambiguous, never a fictitious mean.
      if(direction!=0&&direction!=side)return Prediction.UNKNOWN;
      direction=side;
      if(Math.abs(z-predicted)<distance){distance=Math.abs(z-predicted);closest=p;}
    }
    if(closest==null)return Prediction.UNKNOWN;
    // Curvature changes the extrapolated center as the window grows. Require the
    // proposed surrounding surface to agree across scales before challenging a patch.
    boolean stable=false;
    double at=closest.at(x,y);
    for(Plane p:planes)if(p.scale<closest.scale&&inside(p.hull,x,y)
        && Math.abs(p.at(x,y)-at)<=Math.max(relative*.5*at,3*Math.hypot(p.sigma,closest.sigma))) {
      stable=true;break;
    }
    if(!stable)return Prediction.UNKNOWN;
    if(thin(x,y,z,closest))return Prediction.UNKNOWN;
    double inverse=closest.at(x,y);
    return new Prediction(direction,1/inverse,closest.sigma/(inverse*inverse));
  }

  private Plane[] build(int cx,int cy) {
    var result=new ArrayList<Plane>();
    // Annular samples deliberately exclude the central patch being challenged.
    for(int stride:new int[]{4,8,16}) {
      double[] x=new double[81],y=new double[81],q=new double[81];int n=0;
      for(int dy=-4;dy<=4;dy++)for(int dx=-4;dx<=4;dx++) {
        if(Math.max(Math.abs(dx),Math.abs(dy))<=1)continue;
        int xx=cx+dx*stride,yy=cy+dy*stride;
        if(xx<0||yy<0||xx>=width||yy>=height)continue;
        double z=depth.applyAsDouble(yy*width+xx);
        if(!(z>0)||!Double.isFinite(z))continue;
        x[n]=xx;y[n]=yy;q[n++]=1/z;
      }
      if(n<18)continue;
      double[] sorted=Arrays.copyOf(q,n);Arrays.sort(sorted);
      double tolerance=Math.max(1e-12,sorted[n/2]*Math.max(.0005,relative*.35));
      boolean[] used=new boolean[n];
      for(int layer=0;layer<3;layer++) {
        int best=0;double a=0,b=0,c=0;
        for(int trial=0;trial<64;trial++) {
          int i=(trial*17)%n,j=(trial*31+n/3)%n,k=(trial*43+2*n/3)%n;
          if(used[i]||used[j]||used[k])continue;
          double ux=x[j]-x[i],uy=y[j]-y[i],vx=x[k]-x[i],vy=y[k]-y[i],det=ux*vy-uy*vx;
          if(Math.abs(det)<stride*stride)continue;
          double az=((q[j]-q[i])*vy-uy*(q[k]-q[i]))/det;
          double bz=(ux*(q[k]-q[i])-(q[j]-q[i])*vx)/det,cz=q[i]-az*x[i]-bz*y[i];
          int count=0;for(int t=0;t<n;t++)if(!used[t]&&Math.abs(q[t]-az*x[t]-bz*y[t]-cz)<=tolerance)count++;
          if(count>best){best=count;a=az;b=bz;c=cz;}
        }
        if(best<Math.max(12,(int)Math.ceil(n*.25)))break;
        double mx=0,my=0,mq=0;int count=0;
        for(int i=0;i<n;i++)if(!used[i]&&Math.abs(q[i]-a*x[i]-b*y[i]-c)<=tolerance){mx+=x[i];my+=y[i];mq+=q[i];count++;}
        mx/=count;my/=count;mq/=count;
        double xx=0,xy=0,yy=0,xq=0,yq=0;
        for(int i=0;i<n;i++)if(!used[i]&&Math.abs(q[i]-a*x[i]-b*y[i]-c)<=tolerance) {
          double u=x[i]-mx,v=y[i]-my,d=q[i]-mq;xx+=u*u;xy+=u*v;yy+=v*v;xq+=u*d;yq+=v*d;
        }
        double det=xx*yy-xy*xy;if(det<stride*stride*stride*stride)break;
        a=(xq*yy-yq*xy)/det;b=(yq*xx-xq*xy)/det;c=mq-a*mx-b*my;
        var inliers=new ArrayList<double[]>();double[] residual=new double[n];int m=0;
        for(int i=0;i<n;i++)if(!used[i]&&Math.abs(q[i]-a*x[i]-b*y[i]-c)<=tolerance) {
          used[i]=true;inliers.add(new double[]{x[i],y[i]});residual[m++]=Math.abs(q[i]-a*x[i]-b*y[i]-c);
        }
        if(m<12)continue;
        Arrays.sort(residual,0,m);double sigma=Math.max(mq*.00025,1.4826*residual[m/2]);
        double[] hull=hull(inliers);
        if(hull.length<6)continue;
        // Broad models must actually occupy a 2-D patch, not a line or a narrow edge.
        if(xx/count<stride*stride||yy/count<stride*stride)continue;
        result.add(new Plane(a,b,c,sigma,hull,stride));
      }
    }
    return result.toArray(Plane[]::new);
  }

  private boolean thin(double x,double y,double z,Plane plane) {
    int cx=(int)Math.round(x),cy=(int)Math.round(y),r=3;
    if(cx<r||cy<r||cx>=width-r||cy>=height-r)return false;
    double center=1/z-plane.at(cx,cy),band=Math.max(relative*.5/z,plane.sigma*2);
    boolean[] same=new boolean[49],seen=new boolean[49];int[] queue=new int[49];int tail=1,head=0;queue[0]=24;seen[24]=true;
    for(int dy=-r;dy<=r;dy++)for(int dx=-r;dx<=r;dx++) {
      double d=depth.applyAsDouble((cy+dy)*width+cx+dx);
      same[(dy+r)*7+dx+r]=d>0&&Math.abs(1/d-plane.at(cx+dx,cy+dy)-center)<=band;
    }
    while(head<tail){int p=queue[head++],px=p%7,py=p/7;
      for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
        int xx=px+dx,yy=py+dy;if(xx<0||yy<0||xx>=7||yy>=7)continue;int q=yy*7+xx;
        if(!seen[q]&&same[q]){seen[q]=true;queue[tail++]=q;}
      }
    }
    if(tail<4)return false;
    double xx=0,xy=0,yy=0;boolean reach=false;
    for(int i=1;i<tail;i++){int p=queue[i],dx=p%7-r,dy=p/7-r;xx+=dx*dx;xy+=dx*dy;yy+=dy*dy;if(Math.max(Math.abs(dx),Math.abs(dy))==r)reach=true;}
    double large=(xx+yy+Math.hypot(xx-yy,2*xy))*.5;
    return reach&&large>0&&xx+yy-large<large*.12;
  }

  private static double cross(double[] a,double[] b,double[] c){return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);}
  private static double[] hull(List<double[]> points) {
    points.sort(Comparator.<double[]>comparingDouble(p->p[0]).thenComparingDouble(p->p[1]));
    double[][] h=new double[points.size()*2][];int n=0;
    for(double[] p:points){while(n>=2&&cross(h[n-2],h[n-1],p)<=0)n--;h[n++]=p;}
    int lower=n;
    for(int i=points.size()-2;i>=0;i--){double[] p=points.get(i);while(n>lower&&cross(h[n-2],h[n-1],p)<=0)n--;h[n++]=p;}
    if(n>1)n--;double[] out=new double[2*n];for(int i=0;i<n;i++){out[2*i]=h[i][0];out[2*i+1]=h[i][1];}return out;
  }
  private static boolean inside(double[] h,double x,double y) {
    for(int i=0;i<h.length;i+=2){int j=(i+2)%h.length;double cross=(h[j]-h[i])*(y-h[i+1])-(h[j+1]-h[i+1])*(x-h[i]);if(cross<0)return false;}return true;
  }
}
