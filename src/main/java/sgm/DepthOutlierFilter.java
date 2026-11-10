package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Conservative rejection against a robust local disparity plane (inverse-depth surface).
 * Decisions read an immutable input. Neither smoothing nor sequential/cascading rejection. */
final class DepthOutlierFilter {
  private DepthOutlierFilter() {}
  static boolean enabled(Options o) { return o.depthOutlierFilter.equals("edge-aware"); }
  static final String[] REASONS={"invalid", "border_context", "insufficient_same_gray_support",
      "no_dominant_surface", "ill_conditioned_surface", "within_noise_and_uncertainty",
      "connected_directional_feature", "surface_boundary", "rejected_surface_departure",
      "connected_surface_extent", "uncertainty_preserved_departure"};
  static final class Diagnostics {
    final long[] counts=new long[REASONS.length];
    void add(long[] other) {if(other!=null)for(int i=0;i<counts.length;i++)counts[i]+=other[i];}
    Object json() {
      Map<String,Object> result=new LinkedHashMap<>();
      for(int i=0;i<counts.length;i++)result.put(REASONS[i],counts[i]);
      return result;
    }
  }

  static CensusSgm.Result filter(Options o,CensusSgm.Result result,byte[] gray,
      CensusSgm.Observer observer) throws IOException {
    if (!enabled(o)) return result;
    long began=System.nanoTime();
    Diagnostics diagnostics=new Diagnostics();
    float[] reasons=observer==null?null:new float[result.disparity.length];
    BitSet rejected=mask(o,result.disparity,gray,result.width,result.height,0,result.height,
        result.leftRightError,diagnostics,reasons);
    result.stats.depthOutlierReasons=diagnostics.counts;
    for(int p=rejected.nextSetBit(0);p>=0;p=rejected.nextSetBit(p+1)) {
      result.disparity[p]=Float.NaN;
      if(result.confidence!=null)result.confidence[p]=Float.NaN;
      if(result.rightConfidence!=null)result.rightConfidence[p]=Float.NaN;
    }
    result.stats.depthOutlierRejected=rejected.cardinality();
    result.stats.valid=Math.max(0,result.stats.valid-rejected.cardinality());
    RunTelemetry.record("Edge-aware depth outlier rejection",began);
    if(observer!=null)observer.raster("06_depth_outlier_filtered",result.disparity,result.width,result.height);
    if(observer!=null)observer.raster("07_depth_outlier_reason",reasons,result.width,result.height);
    report(o,result.stats.depthOutlierRejected,diagnostics);
    return result;
  }

  static void report(Options o,long rejected) throws IOException {
    report(o,rejected,null);
  }
  static void report(Options o,long rejected,Diagnostics diagnostics) throws IOException {
    if(!enabled(o))return;
    System.out.println("Edge-aware depth outlier filter: rejected "+rejected+" surface departures; survivors unchanged");
    if(o.stereoReportDirectory!=null) {
      Files.createDirectories(o.stereoReportDirectory);
      Json.write(o.stereoReportDirectory.resolve("depth_outlier_filter.json"),Json.object(
          "filter","edge-aware","version",3,"rejected",rejected,"radius",o.depthOutlierRadius,
          "deviation_disparity_pixels",o.depthOutlierDeviation,
          "grayscale_edge_threshold",o.depthOutlierEdgeThreshold,"moves_survivors",false,
          "decision_counts",diagnostics==null?null:diagnostics.json(),
          "reason_raster_codes",Arrays.asList(REASONS),
          "uncertainty_model","Disparity-plane MAD, prediction leverage, sampling floor and available left/right error; not calibration covariance",
          "camera_checks","Independent-camera agreement and coherent free space checked downstream during fusion"));
    }
  }

  /** Rows outside the requested core are context only, including when used on windowed strips. */
  static BitSet mask(Options o,float[] depth,byte[] gray,int w,int h,int from,int to) {
    return mask(o,depth,gray,w,h,from,to,null,new Diagnostics(),null);
  }
  static BitSet mask(Options o,float[] depth,byte[] gray,int w,int h,int from,int to,
      float[] lrError,Diagnostics diagnostics,float[] reasons) {
    BitSet rejected=new BitSet();
    if(!enabled(o))return rejected;
    if((long)w*h!=depth.length || gray.length!=depth.length || from<0 || to>h || from>to)
      throw new IllegalArgumentException("Invalid depth outlier raster/core");
    // Decisions read only the unfiltered input, so pixels are independent: decided on the GPU
    // (same double arithmetic, see GpuDepthOutlier) or in parallel rows on the CPU, then counted
    // and applied serially in raster order (identical result). Windowed callers supply bounded strips.
    byte[] codes=GpuDepthOutlier.codes(o,depth,gray,w,h,from,to,lrError);
    if(codes==null)codes=codes(o,depth,gray,w,h,from,to,lrError,1);
    for(int y=from;y<to;y++)
      for(int x=0;x<w;x++) {
        int p=y*w+x, reason=codes[(y-from)*w+x];
        diagnostics.counts[reason]++;
        if(reasons!=null)reasons[p]=reason;
        if(reason==8)rejected.set(p);
      }
    return rejected;
  }

  /** CPU decision codes of rows from, from+step, ... below to (other rows stay 0), as mask() uses them. */
  static byte[] codes(Options o,float[] depth,byte[] gray,int w,int h,int from,int to,
      float[] lrError,int step) {
    int r=o.depthOutlierRadius;
    byte[] codes=new byte[(to-from)*w];
    java.util.stream.IntStream.range(0,(to-from+step-1)/step).parallel().forEach(k->{
      int y=from+k*step;
      Scratch scratch=new Scratch((2*r+1)*(2*r+1));
      for(int x=0;x<w;x++)codes[(y-from)*w+x]=(byte)code(o,depth,gray,w,h,x,y,lrError,scratch);
    });
    return codes;
  }

  /** CPU decision code of one pixel (GpuDepthOutlier returns pixels that reach structure()). */
  static int code(Options o,float[] depth,byte[] gray,int w,int h,int x,int y,float[] lrError) {
    int r=o.depthOutlierRadius;
    return code(o,depth,gray,w,h,x,y,lrError,new Scratch((2*r+1)*(2*r+1)));
  }

  private static int code(Options o,float[] depth,byte[] gray,int w,int h,int x,int y,
      float[] lrError,Scratch scratch) {
    int p=y*w+x,r=o.depthOutlierRadius;
    return !Float.isFinite(depth[p])?0:x<r||x>=w-r||y<r||y>=h-r?1:
        decision(o,depth,gray,w,x,y,scratch,lrError==null?Double.NaN:lrError[p]);
  }

  private static final class Scratch {
    final double[] x,y,z,residual;
    final boolean[] visited;
    final int[] queue;
    int n;
    Scratch(int n){x=new double[n];y=new double[n];z=new double[n];residual=new double[n];
      visited=new boolean[n];queue=new int[n];}
  }

  private static int decision(Options o,float[] d,byte[] gray,int w,int x,int y,Scratch s,double lr) {
    int p=y*w+x,g=gray[p]&255,r=o.depthOutlierRadius,sectors=0;
    double center=d[p],tolerance=Math.max(.25,o.depthOutlierDeviation*.35);
    s.n=0;
    for(int dy=-r;dy<=r;dy++)for(int dx=-r;dx<=r;dx++) {
      if(dx==0 && dy==0)continue;
      int q=p+dy*w+dx;
      if(!Float.isFinite(d[q]) || Math.abs((gray[q]&255)-g)>o.depthOutlierEdgeThreshold)continue;
      int i=s.n++;s.x[i]=dx;s.y[i]=dy;s.z[i]=d[q];sectors|=sector(dx,dy);
    }
    int n=s.n;
    if(n<12 || Integer.bitCount(sectors)<6)return 2;
    int required=(int)Math.ceil(n*.65),best=0;
    double bestA=0,bestB=0,bestC=0;
    // Deterministic robust plane hypotheses. Non-collinear, spatially spread support
    // is required again after refinement; a depth edge must not become a smoothing plane.
    final double[] sx=s.x,sy=s.y,sz=s.z;
    for(int trial=0;trial<48;trial++) {
      int i=(trial*7)%n,j=(trial*13+n/3)%n,k=(trial*19+2*n/3)%n;
      double ux=sx[j]-sx[i],uy=sy[j]-sy[i],vx=sx[k]-sx[i],vy=sy[k]-sy[i];
      double determinant=ux*vy-uy*vx;
      if(Math.abs(determinant)<2)continue;
      double uz=sz[j]-sz[i],vz=sz[k]-sz[i];
      double a=(uz*vy-uy*vz)/determinant,b=(ux*vz-uz*vx)/determinant,c=sz[i]-a*sx[i]-b*sy[i];
      int support=0;
      // Same expression and comparison; counted without a data-dependent branch. A trial only
      // matters if its support exceeds best, so counting stops once that is impossible (exact).
      for(int t=0;t<n;) {
        int end=Math.min(n,t+8);
        for(;t<end;t++)support+=Math.abs(sz[t]-a*sx[t]-b*sy[t]-c)<=tolerance?1:0;
        if(support+(n-t)<=best)break;
      }
      if(support>best){best=support;bestA=a;bestB=b;bestC=c;}
      if(best==n)break;
    }
    if(best<required)return 3;
    double mx=0,my=0,mz=0;
    for(int i=0;i<n;i++)if(Math.abs(s.z[i]-bestA*s.x[i]-bestB*s.y[i]-bestC)<=tolerance) {
      mx+=s.x[i];my+=s.y[i];mz+=s.z[i];
    }
    mx/=best;my/=best;mz/=best;
    double xx=0,xy=0,yy=0,xz=0,yz=0;
    for(int i=0;i<n;i++)if(Math.abs(s.z[i]-bestA*s.x[i]-bestB*s.y[i]-bestC)<=tolerance) {
      double dx=s.x[i]-mx,dy=s.y[i]-my,dz=s.z[i]-mz;
      xx+=dx*dx;xy+=dx*dy;yy+=dy*dy;xz+=dx*dz;yz+=dy*dz;
    }
    double det=xx*yy-xy*xy;
    if(det<1e-6)return 4;
    double a=(xz*yy-yz*xy)/det,b=(yz*xx-xz*xy)/det,c=mz-a*mx-b*my;
    int support=0;sectors=0;
    for(int i=0;i<n;i++) {
      double residual=Math.abs(s.z[i]-a*s.x[i]-b*s.y[i]-c);
      if(residual<=tolerance) {
        s.residual[support++]=residual;sectors|=sector((int)s.x[i],(int)s.y[i]);
      }
    }
    if(support<required || Integer.bitCount(sectors)<6)return 7;
    double departure=Math.abs(center-c);
    // band >= depthOutlierDeviation always (noise, leverage and measurement are finite and
    // non-negative), so a departure within the deviation is code 5 whatever the scatter is.
    // Skips the median sort for most pixels; identical decisions.
    if(departure<=o.depthOutlierDeviation)return 5;
    // Estimate scatter from all usable neighbours, not the truncated RANSAC inliers:
    // truncation would make a noisy surface look artificially precise.
    for(int i=0;i<n;i++)s.residual[i]=Math.abs(s.z[i]-a*s.x[i]-b*s.y[i]-c);
    Arrays.sort(s.residual,0,n);
    double noise=1.4826*s.residual[n/2];
    double leverage=1.0/support+(yy*mx*mx-2*xy*mx*my+xx*my*my)/det;
    double measurement=Double.isFinite(lr)?Math.max(.25,Math.min(2,Math.abs(lr)*.5)):.25;
    double band=Math.max(o.depthOutlierDeviation,3*Math.sqrt(noise*noise*(1+leverage)+measurement*measurement));
    if(departure<=band)return departure>o.depthOutlierDeviation?10:5;
    int structure=structure(s,center,a,b,Math.max(.5,Math.min(band*.5,tolerance*1.5)),r);
    if(structure!=0)return structure;
    return 8;
  }

  /** Connected same-layer support must span the context along a narrow line through
   * the center. Compact blobs cannot protect themselves by supplying two neighbours. */
  private static int structure(Scratch s,double center,double a,double b,double band,int r) {
    Arrays.fill(s.visited,false);
    int head=0,tail=0;
    for(int i=0;i<s.n;i++)if(Math.max(Math.abs(s.x[i]),Math.abs(s.y[i]))<=1
        && Math.abs(s.z[i]-a*s.x[i]-b*s.y[i]-center)<=band) {
      s.visited[i]=true;s.queue[tail++]=i;
    }
    while(head<tail) {
      int i=s.queue[head++];
      for(int j=0;j<s.n;j++)if(!s.visited[j] && Math.abs(s.x[j]-s.x[i])<=1
          && Math.abs(s.y[j]-s.y[i])<=1 && Math.abs(s.z[j]-a*s.x[j]-b*s.y[j]-center)<=band) {
        s.visited[j]=true;s.queue[tail++]=j;
      }
    }
    if(tail<r)return 0;
    double xx=0,xy=0,yy=0,reach=0;int boundary=0;
    for(int k=0;k<tail;k++){int i=s.queue[k];xx+=s.x[i]*s.x[i];xy+=s.x[i]*s.y[i];yy+=s.y[i]*s.y[i];
      double distance=Math.max(Math.abs(s.x[i]),Math.abs(s.y[i]));
      reach=Math.max(reach,distance);if(distance==r)boundary++;}
    // A connected sheet extending out of the local window can be a roof corner.
    // Local evidence alone cannot distinguish it from a broad duplicate surface.
    if(tail>=r*r && boundary>=r+1)return 9;
    double large=(xx+yy+Math.hypot(xx-yy,2*xy))*.5,small=xx+yy-large;
    if(reach<r || large<=0 || small>large*.12)return 0;
    double angle=.5*Math.atan2(2*xy,xx-yy),nx=-Math.sin(angle),ny=Math.cos(angle);
    for(int k=0;k<tail;k++){int i=s.queue[k];if(Math.abs(nx*s.x[i]+ny*s.y[i])>.8)return 0;}
    return 6;
  }

  private static int sector(int x,int y) {
    if(y<0)return x<0?1:x==0?2:4;
    if(y==0)return x<0?8:16;
    return x<0?32:x==0?64:128;
  }

  /** 0=unknown/edge, 1=agreement, 2=observed free space, 3=nearer occluder.
   * Missing depths and discontinuities never cast a contradiction vote. */
  static int cameraVote(double candidate,double low,double high,int count,double sigma,double footprint) {
    return cameraVote(candidate,(low+high)*.5,low,high,count,sigma,footprint,0);
  }
  static int cameraVote(double candidate,double center,double low,double high,int count,
      double sigma,double footprint,double relative) {
    if(count<7 || !(low>0) || !Double.isFinite(high) || !Double.isFinite(sigma)
        || !(footprint>0))return 0;
    double precise=Math.max(3*footprint,relative*Math.min(candidate,low));
    if(high-low>2*precise)return 0;
    double band=3*Math.hypot(Math.max(0,sigma),footprint);
    if(candidate<low-band)return 2;
    if(candidate>high+band)return 3;
    // Uncertainty weakens a contradiction; it does not manufacture positive evidence.
    return center>0&&Math.abs(candidate-center)<=precise?1:0;
  }
  /** cameraVote with sigma = hypot(sourceSigma, otherSigma), evaluated only when it is used. */
  static int cameraVote(double candidate,double center,double low,double high,int count,
      double sourceSigma,double otherSigma,double footprint,double relative) {
    if(count<7 || !(low>0) || !Double.isFinite(high) || !(footprint>0))return 0;
    double precise=Math.max(3*footprint,relative*Math.min(candidate,low));
    if(high-low>2*precise)return 0;
    // band = 3*hypot(hypot(source, other), footprint) is within a few ulps of 3*sqrt(sum of
    // squares). With a 1e-12 relative margin, [lowBand, highBand] surely contains the exact band;
    // rounding of low-band and high+band is monotonic, so outside the narrow ambiguous zone the
    // comparisons below decide exactly as with the exact band. Ambiguous or extreme values use
    // the exact computation.
    if(Double.isFinite(sourceSigma)&&Double.isFinite(otherSigma)) {
      double t=Math.sqrt(sourceSigma*sourceSigma+otherSigma*otherSigma+footprint*footprint);
      if(t>1e-100&&t<1e100) {
        double lowBand=3*t*(1-1e-12),highBand=3*t*(1+1e-12);
        if(candidate<low-highBand)return 2;
        if(candidate>=low-lowBand) {
          if(candidate>high+highBand)return 3;
          if(candidate<=high+lowBand)return center>0&&Math.abs(candidate-center)<=precise?1:0;
        }
      }
    }
    return cameraVote(candidate,center,low,high,count,Math.hypot(sourceSigma,otherSigma),footprint,relative);
  }
  static boolean cameraReject(int support,int freeSpace) {
    return freeSpace>=2 && freeSpace>support;
  }
}
