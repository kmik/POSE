package sgm;

import java.io.*;
import java.util.*;
import java.util.concurrent.*;
import org.opencv.core.*;

/** Opt-in approximate, coarse-guided, GPU-resident SGM tiles. No aggregate disk storage. */
final class AerialTileStereo {
  record Pixels(int width,int height,byte[] gray,byte[] color,byte[] mask) {}
  interface Rectifier {
    Pixels read(boolean right,double x,double y,int w,int h,int scale) throws IOException;
  }
  record Core(int x,int y,int width,int height) {}
  record Plan(Core core,int minimum,int maximum,int shift,int halo,int x,int y,int width,int height,
      boolean full) {}
  record Prepared(Plan plan,Pixels left,Pixels right) {}
  record Match(Prepared prepared,CensusSgm.Result result,byte[] boundary) {
    float disparity(int x,int y) {
      Plan p=prepared.plan;
      float d=result.disparity[(y-p.y)*p.width+x-p.x];
      return Float.isFinite(d) ? d+p.shift : Float.NaN;
    }
  }
  interface Sink {
    /** Fraction of overlapping valid predictions that disagree; missing data is neutral. */
    double seamError(Match match) throws IOException;
    void accept(Match match,boolean rejectSeam) throws IOException;
  }
  static final class TileTooLarge extends IOException {
    TileTooLarge(String message) { super(message); }
  }
  private final Options o;
  private final Rectifier rectifier;
  private final int w,h;
  private final GpuSgm.Device device;
  private final long gpuBudget;
  private final long hostBudget;
  private int factor,coarseWidth,coarseHeight,minimum,maximum;
  private float[] coarse;
  long tiles,retries,fullSearches,subdivisions,seamRejected,preparedAhead;
  long peakTilePixels,peakEstimatedGpuBytes,peakEstimatedHostBytes;
  long coarseNanos,prepareNanos,matchNanos;
  long depthOutlierRejections;
  final DepthOutlierFilter.Diagnostics depthOutlierDiagnostics=new DepthOutlierFilter.Diagnostics();

  AerialTileStereo(Options options,Rectifier rectifier,int width,int height) {
    o=options;this.rectifier=rectifier;w=width;h=height;
    device=GpuSgm.selectedDevice(o);
    gpuBudget=GpuSgm.budget(device,o);
    hostBudget=MemorySafety.snapshot().allowance(o.memoryMb*1048576L);
  }

  Object report() {
    return Json.object("pipeline","aerial-tiled","matching_backend","OpenCL GPU",
        "approximate_tile_paths",true,"coarse_scale",factor,"coarse_width",coarseWidth,
        "coarse_height",coarseHeight,"pair_min_disparity",minimum,
        "pair_max_disparity",maximum,"tiles",tiles,"retries",retries,
        "full_interval_tiles",fullSearches,"subdivisions",subdivisions,
        "seam_rejected_tiles",seamRejected,"prepared_ahead",preparedAhead,
        "peak_tile_pixels",peakTilePixels,"peak_estimated_gpu_bytes",peakEstimatedGpuBytes,
        "peak_estimated_tile_host_bytes",peakEstimatedHostBytes,"aggregate_disk_bytes",0,
        "coarse_seconds",coarseNanos/1e9,"preparation_wait_seconds",prepareNanos/1e9,
        "matching_seconds",matchNanos/1e9,"depth_outlier_filter",o.depthOutlierFilter,
        "depth_outliers_rejected_in_tile_solves",depthOutlierRejections,
        "depth_outlier_decision_counts_in_tile_solves",DepthOutlierFilter.enabled(o)?depthOutlierDiagnostics.json():null);
  }

  void run(Sink sink) throws IOException {
    long began=System.nanoTime();
    coarse();coarseNanos=System.nanoTime()-began;
    Deque<Plan> queue=new ArrayDeque<>();
    for(int y=0;y<h;y+=o.aerialTileSize)
      for(int x=0;x<w;x+=o.aerialTileSize)
        enqueue(queue,new Core(x,y,Math.min(o.aerialTileSize,w-x),Math.min(o.aerialTileSize,h-y)));
    // Exactly one independently decoded tile is prepared ahead. Native codecs and Mats
    // are confined to the preparing task; the GPU session remains on the consumer thread.
    ExecutorService worker=Executors.newSingleThreadExecutor(r->new Thread(r,"aerial-rectification"));
    Future<Prepared> pending=null;
    Plan pendingPlan=null;
    try(var progress=WorkProgress.start("Aerial GPU tiles",0,"tiles")) {
      if(!queue.isEmpty()) {pendingPlan=queue.removeFirst();pending=submit(worker,pendingPlan);}
      while(pending!=null) {
        long clock=System.nanoTime();
        Prepared prepared;
        try { prepared=join(pending); }
        catch(TileTooLarge failure) {
          subdivisions++;
          var children=split(pendingPlan.core);
          for(int i=children.size()-1;i>=0;i--)
            addFirst(queue,plan(children.get(i),pendingPlan.minimum,pendingPlan.maximum,pendingPlan.full));
          pendingPlan=queue.removeFirst();pending=submit(worker,pendingPlan);continue;
        }
        prepareNanos+=System.nanoTime()-clock;
        pending=null;
        if(!queue.isEmpty()) { pendingPlan=queue.removeFirst();pending=submit(worker,pendingPlan); preparedAhead++; }
        Plan plan=prepared.plan;
        boolean rejectSeam=false;
        Match match;
        for(;;) {
          clock=System.nanoTime();
          match=match(prepared);matchNanos+=System.nanoTime()-clock;
          double seam=sink.seamError(match);
          boolean weak=weak(match);
          if(!plan.full && (weak || seam>.05)) {
            retries++;
            if(retries<=5)System.out.printf(Locale.ROOT,
                "Aerial tile (%d,%d): restoring pair interval (weak=%s, seam disagreement=%.1f%%)%n",
                plan.core.x,plan.core.y,weak,100*seam);
            Plan expanded=plan(plan.core,minimum,maximum,true);
            if(!fits(expanded)) {
              // Restore ordering after an already submitted next tile. A smaller spatial
              // core may fit the full interval; never silently narrow or use CPU matching.
              if(pending!=null) {
                try {join(pending);}catch(TileTooLarge ignored) {}
                queue.addFirst(pendingPlan);pending=null;
              }
              List<Core> split=split(plan.core);
              for(int i=split.size()-1;i>=0;i--) addFirst(queue,plan(split.get(i),minimum,maximum,true));
              subdivisions++;match=null;break;
            }
            try {prepared=prepare(expanded);plan=expanded;continue;}
            catch(TileTooLarge footprint) {
              if(pending!=null) {
                try {join(pending);}catch(TileTooLarge ignored) {}
                queue.addFirst(pendingPlan);pending=null;
              }
              var children=split(plan.core);
              for(int i=children.size()-1;i>=0;i--)addFirst(queue,plan(children.get(i),minimum,maximum,true));
              subdivisions++;match=null;break;
            }
          }
          rejectSeam=seam>.05;
          if(rejectSeam)seamRejected++;
          break;
        }
        if(match!=null) { sink.accept(match,rejectSeam);tiles++;progress.update(tiles); }
        if(pending==null && !queue.isEmpty()){pendingPlan=queue.removeFirst();pending=submit(worker,pendingPlan);}
      }
      progress.done();
    } finally {
      if(pending!=null)pending.cancel(true);
      worker.shutdownNow();
      try { if(!worker.awaitTermination(30,TimeUnit.SECONDS))
        throw new IOException("Aerial tile decoder did not stop"); }
      catch(InterruptedException e) { Thread.currentThread().interrupt(); }
    }
  }

  private Future<Prepared> submit(ExecutorService worker,Plan p) {
    return worker.submit(()->prepare(p));
  }
  private static Prepared join(Future<Prepared> future) throws IOException {
    try{return future.get();}
    catch(InterruptedException e){Thread.currentThread().interrupt();throw new IOException("Aerial processing interrupted",e);}
    catch(ExecutionException e){
      if(e.getCause() instanceof IOException x)throw x;
      if(e.getCause() instanceof RuntimeException x)throw x;
      if(e.getCause() instanceof Error x)throw x;
      throw new IOException(e.getCause());
    }
  }

  private void coarse() throws IOException {
    factor=1;
    while(Math.ceilDiv(Math.max(w,h),factor)>o.aerialCoarseDimension)factor*=2;
    coarseWidth=Math.ceilDiv(w,factor);coarseHeight=Math.ceilDiv(h,factor);
    if(Math.min(coarseWidth,coarseHeight)<o.census+4)
      throw new IllegalArgumentException("Aerial coarse image is too narrow; raise --aerial-coarse-dimension");
    double origin=(factor-1)*.5;
    Pixels left=rectifier.read(false,origin,origin,coarseWidth,coarseHeight,factor);
    Pixels right=rectifier.read(true,origin,origin,coarseWidth,coarseHeight,factor);
    Options level=matchingOptions();
    level.minDisparity=Math.floorDiv(o.minDisparity,factor);
    level.disparities=Math.max(3,Math.ceilDiv(o.minDisparity+o.disparities-1,factor)-level.minDisparity+1);
    if(!o.disparitiesExplicit) {
      Mat a=new Mat(coarseHeight,coarseWidth,CvType.CV_8U),b=new Mat(),ma=new Mat(),mb=new Mat();
      try {
        a.put(0,0,left.gray);b.create(a.size(),a.type());b.put(0,0,right.gray);
        ma.create(a.size(),a.type());ma.put(0,0,left.mask);mb.create(a.size(),a.type());mb.put(0,0,right.mask);
        var range=DisparityRange.estimate(a,b,ma,mb,level,List.of());
        level.minDisparity=range.minimum;level.disparities=range.count;
      } finally { a.release();b.release();ma.release();mb.release(); }
    }
    CensusSgm.Result result=null;
    for(int attempt=0;attempt<5;attempt++) {
      ensureBudget(coarseWidth,coarseHeight,level);
      result=new CensusSgm(level).compute(left.gray,right.gray,left.color,right.color,
          coarseWidth,coarseHeight,left.mask,right.mask);
      if(o.disparitiesExplicit)break;
      var next=DisparityRange.expansion(level,level,result.stats,coarseWidth,attempt);
      if(next==null)break;
      if(attempt==4)throw new IOException("Aerial coarse search still has boundary pressure; supply a wider --min-disparity/--disparities interval");
      level.minDisparity=next.minimum();level.disparities=next.count();
    }
    coarse=result.disparity;
    minimum=o.disparitiesExplicit ? o.minDisparity
        : Math.max(-w+1,level.minDisparity*factor-o.aerialDisparityMargin);
    maximum=o.disparitiesExplicit ? o.minDisparity+o.disparities-1
        : Math.min(w-1,(level.minDisparity+level.disparities)*factor+o.aerialDisparityMargin);
    System.out.printf(Locale.ROOT,"Aerial coarse %dx%d (1/%d): pair interval [%d,%d]; GPU-resident tiles, no aggregate spill%n",
        coarseWidth,coarseHeight,factor,minimum,maximum);
  }

  Plan predicted(Core core) {
    int min=Integer.MAX_VALUE,max=Integer.MIN_VALUE,valid=0,total=0;
    int x0=Math.max(0,core.x/factor-1),x1=Math.min(coarseWidth-1,(core.x+core.width)/factor+1);
    int y0=Math.max(0,core.y/factor-1),y1=Math.min(coarseHeight-1,(core.y+core.height)/factor+1);
    for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++) {
      total++;float d=coarse[y*coarseWidth+x];
      if(!Float.isFinite(d))continue;
      valid++;min=Math.min(min,(int)Math.floor(d*factor));max=Math.max(max,(int)Math.ceil(d*factor));
    }
    boolean full=valid<Math.max(4,total/4);
    if(full)return plan(core,minimum,maximum,true);
    return plan(core,Math.max(minimum,min-o.aerialDisparityMargin),
        Math.min(maximum,max+o.aerialDisparityMargin),false);
  }

  private Plan plan(Core c,int min,int max,boolean full) {
    if(max-min<2){min=Math.max(minimum,min-1);max=Math.min(maximum,min+2);}
    // Existing reciprocal/conditioning filters treat zero disparity as invalid. Keep
    // the local search strictly positive even when the physical/global disparity is
    // negative or its prediction happens to equal the crop offset.
    int shift=min-Math.max(8,Math.max(o.census,o.correlationWindow));
    int overlap=DepthOutlierFilter.enabled(o)
        ? Math.max(o.aerialOverlap,o.depthOutlierRadius+Math.max(o.census,o.correlationWindow)/2+1)
        : o.aerialOverlap;
    int halo=overlap+max-shift+Math.max(o.census,o.correlationWindow);
    return new Plan(c,min,max,shift,halo,c.x-halo,c.y-overlap,
        c.width+2*halo,c.height+2*overlap,full);
  }
  private void enqueue(Deque<Plan> queue,Core c) throws IOException {
    Plan p=predicted(c);
    if(fits(p))queue.addLast(p);
    else {subdivisions++;for(Core child:split(c))enqueue(queue,child);}
  }
  private void addFirst(Deque<Plan> queue,Plan p) throws IOException {
    if(fits(p)){queue.addFirst(p);return;}
    subdivisions++;var children=split(p.core);
    for(int i=children.size()-1;i>=0;i--)
      addFirst(queue,plan(children.get(i),p.minimum,p.maximum,p.full));
  }
  private List<Core> split(Core c) throws IOException {
    if(c.width<=32 && c.height<=32)
      throw new TileTooLarge("Even a 32-pixel core with the restored disparity interval exceeds the GPU/host tile budget; reduce resolution or tighten a justified pair interval");
    if(c.width>=c.height && c.width>32) {
      int half=c.width/2;
      return List.of(new Core(c.x,c.y,half,c.height),new Core(c.x+half,c.y,c.width-half,c.height));
    }
    int half=c.height/2;
    return List.of(new Core(c.x,c.y,c.width,half),new Core(c.x,c.y+half,c.width,c.height-half));
  }
  private boolean fits(Plan p) {
    Options m=matchingOptions();m.minDisparity=p.minimum-p.shift;m.disparities=p.maximum-p.minimum+1;
    return fits(p.width,p.height,m);
  }
  private boolean fits(int width,int height,Options m) {
    long n=(long)width*height,cells=n*m.disparities;
    long host=n*(240L+32*((m.census*m.census-1+63)/64));
    return n<Integer.MAX_VALUE/24 && cells<Integer.MAX_VALUE-8L
        && host<=hostBudget/3
        && GpuBandedStereo.deviceBytes(m,width,height,cells)<=gpuBudget*4/5
        && 24*n<=device.allocation()
        && (2L*m.disparities+2)*4<=device.localMemory();
  }
  private void ensureBudget(int width,int height,Options m) throws TileTooLarge {
    if(!fits(width,height,m))throw new TileTooLarge("Aerial GPU tile/coarse volume exceeds budget; reduce tile size/coarse dimension or resolution");
    long n=(long)width*height;
    peakTilePixels=Math.max(peakTilePixels,n);
    peakEstimatedGpuBytes=Math.max(peakEstimatedGpuBytes,GpuBandedStereo.deviceBytes(m,width,height,n*m.disparities));
    peakEstimatedHostBytes=Math.max(peakEstimatedHostBytes,n*(240L+32*((m.census*m.census-1+63)/64)));
  }
  private Prepared prepare(Plan p) throws IOException {
    if(Thread.currentThread().isInterrupted())throw new IOException("Aerial decode interrupted");
    if(!fits(p))throw new TileTooLarge("Restored aerial tile interval exceeds budget");
    return new Prepared(p,rectifier.read(false,p.x,p.y,p.width,p.height,1),
        rectifier.read(true,p.x-p.shift,p.y,p.width,p.height,1));
  }
  private Match match(Prepared prepared) throws IOException {
    Plan p=prepared.plan;
    if(p.full)fullSearches++;
    Options m=matchingOptions();m.minDisparity=p.minimum-p.shift;m.disparities=p.maximum-p.minimum+1;
    // The global coarse map bounds each tile; the existing small TSGM pyramid then
    // packs per-pixel bands inside it. Admission still assumes the full tile interval,
    // so every band-edge retry can remain on the GPU without aggregate spilling.
    m.tsgmLevels=Math.min(2,o.tsgmLevels);
    ensureBudget(p.width,p.height,m);
    var stereo=new CensusSgm(m);stereo.captureSearchBoundary=true;
    var result=stereo.compute(prepared.left.gray,prepared.right.gray,
        prepared.left.color,prepared.right.color,p.width,p.height,prepared.left.mask,prepared.right.mask);
    DepthOutlierFilter.filter(m,result,prepared.left.gray,null);
    depthOutlierRejections+=result.stats.depthOutlierRejected;
    depthOutlierDiagnostics.add(result.stats.depthOutlierReasons);
    PointConfidence.stereo(result,prepared.left.gray,prepared.right.gray,m);
    return new Match(prepared,result,stereo.lastSearchBoundary);
  }
  private boolean weak(Match match) {
    Plan p=match.prepared.plan;Core c=p.core;int available=0,valid=0,boundary=0;
    for(int y=c.y;y<c.y+c.height;y++)for(int x=c.x;x<c.x+c.width;x++) {
      int i=(y-p.y)*p.width+x-p.x;
      if(match.prepared.left.mask[i]==0)continue;
      available++;
      if(Float.isFinite(match.result.disparity[i]))valid++;
      // Bit 4 means "searched the full interval", not a boundary hit.
      if(match.boundary!=null && (match.boundary[i]&11)!=0)boundary++;
    }
    return available>0 && (valid<available/10 || boundary>Math.max(3,available/100));
  }
  private Options matchingOptions() {
    Options m=o.copy();m.compute="gpu";m.pairwiseSearch="tsgm";m.tsgmLevels=0;
    m.tsgmSeeds=null;m.tiled=false;m.pyramidLevels=1;m.stereoPipeline=false;
    m.stereoReportDirectory=null;m.stereoSelection=null;
    return m;
  }
}
