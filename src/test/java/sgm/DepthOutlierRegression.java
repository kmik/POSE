package sgm;

import java.nio.file.*;
import java.util.*;

/** Independent surface controls, immutable decisions, strip equivalence and default compatibility. */
public final class DepthOutlierRegression {
  private static int checks;
  private static void check(boolean ok,String message) {
    checks++;if(!ok)throw new AssertionError(message);
  }
  public static void main(String[] args)throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root=Files.createTempDirectory(Path.of("target"),"depth-outlier-regression-");
    System.out.println("PASS depth outlier checks="+run(root)+"; fixtures="+root);
  }
  static int run(Path root)throws Exception {
    Files.createDirectories(root);checks=0;
    Options o=TestSupport.options();o.depthOutlierFilter="edge-aware";
    int w=83,h=67,n=w*h;byte[] gray=new byte[n];Arrays.fill(gray,(byte)128);
    float[] plane=new float[n];
    for(int y=0;y<h;y++)for(int x=0;x<w;x++)plane[y*w+x]=(float)(25+.08*x-.04*y);
    float[] spikes=plane.clone();int first=24*w+30,second=42*w+56;
    spikes[first]+=5;spikes[second]-=5;
    var mask=DepthOutlierFilter.mask(o,spikes,gray,w,h,0,h);
    check(mask.cardinality()==2 && mask.get(first) && mask.get(second),"Both nearer/farther isolated spikes rejected");
    var result=new CensusSgm.Result(w,h,spikes.clone());result.stats.valid=n;
    final float[][] reasons={null};
    DepthOutlierFilter.filter(o,result,gray,(name,values,width,height)-> {
      if(name.equals("07_depth_outlier_reason"))reasons[0]=values.clone();
    });
    check(reasons[0]!=null && reasons[0][first]==8 && reasons[0][second]==8,"Diagnostic raster records individual rejection reasons");
    check(result.stats.depthOutlierRejected==2 && result.stats.valid==n-2,"Accurate rejection/valid counters");
    for(int i=0;i<n;i++)check(mask.get(i)?Float.isNaN(result.disparity[i]):
        Float.floatToRawIntBits(result.disparity[i])==Float.floatToRawIntBits(spikes[i]),"Survivors never move");
    check(DepthOutlierFilter.mask(o,plane,gray,w,h,0,h).isEmpty(),"Sloped plane retained");
    for(int offset:new int[]{-1000,1000}) {
      float[] shifted=spikes.clone();for(int i=0;i<n;i++)shifted[i]+=offset;
      check(mask.equals(DepthOutlierFilter.mask(o,shifted,gray,w,h,0,h)),"Signed/crop-offset invariant decisions");
    }
    float[] edge=plane.clone();
    for(int y=0;y<h;y++)for(int x=w/2;x<w;x++)edge[y*w+x]+=12;
    check(DepthOutlierFilter.mask(o,edge,gray,w,h,0,h).isEmpty(),"Roof/depth discontinuity preserved");
    float[] corner=plane.clone();
    for(int y=30;y<h;y++)for(int x=40;x<w;x++)corner[y*w+x]+=12;
    check(DepthOutlierFilter.mask(o,corner,gray,w,h,0,h).isEmpty(),"Convex roof corner retained even without image contrast");
    float[] wire=plane.clone();for(int x=10;x<73;x++)wire[33*w+x]+=9;
    check(DepthOutlierFilter.mask(o,wire,gray,w,h,0,h).isEmpty(),"One-pixel thin wire and endpoints preserved");
    clusteredAndUncertain(o,plane,gray,w,h);
    cameraVotes();
    patchContext();
    cameraEvidence(root);
    referencePatchEvidence(root);
    imageCameraEvidence(root);
    targetedPatchEvidence(root);
    float[] sparse=new float[n];Arrays.fill(sparse,Float.NaN);sparse[first]=60;
    check(DepthOutlierFilter.mask(o,sparse,gray,w,h,0,h).isEmpty(),"Sparse/unknown region left alone");
    byte[] imageEdge=gray.clone();imageEdge[first]=(byte)255;imageEdge[second]=0;
    check(DepthOutlierFilter.mask(o,spikes,imageEdge,w,h,0,h).isEmpty(),"High contrast details protected");
    float[] vegetation=new float[n];Random random=new Random(99);
    for(int i=0;i<n;i++)vegetation[i]=20+random.nextFloat()*80;
    check(DepthOutlierFilter.mask(o,vegetation,gray,w,h,0,h).isEmpty(),"Mixed rough depths are not flattened");
    for(int strip:new int[]{1,4,11}) {
      BitSet assembled=new BitSet();
      for(int y=0;y<h;y+=strip) {
        int top=Math.max(0,y-o.depthOutlierRadius),bottom=Math.min(h,y+strip+o.depthOutlierRadius);
        var local=DepthOutlierFilter.mask(o,Arrays.copyOfRange(spikes,top*w,bottom*w),
            Arrays.copyOfRange(gray,top*w,bottom*w),w,bottom-top,y-top,Math.min(h,y+strip)-top);
        for(int p=local.nextSetBit(0);p>=0;p=local.nextSetBit(p+1))assembled.set(top*w+p);
      }
      check(mask.equals(assembled),"Windowed strip/halo equivalence at strip height "+strip);
    }
    windowed(root,o,spikes,gray,mask,w,h);
    defaults(root);
    return checks;
  }

  private static void clusteredAndUncertain(Options o,float[] plane,byte[] gray,int w,int h) {
    float[] cluster=plane.clone();
    for(int y=28;y<30;y++)for(int x=36;x<38;x++)cluster[y*w+x]+=5;
    var diagnostic=new DepthOutlierFilter.Diagnostics();
    var mask=DepthOutlierFilter.mask(o,cluster,gray,w,h,0,h,null,diagnostic,null);
    check(mask.cardinality()==4,"A compact 2x2 cluster cannot preserve itself with two supporters");
    check(diagnostic.counts[8]==4 && Arrays.stream(diagnostic.counts).sum()==(long)w*h,"Every pixel has a recorded decision");
    float[] diagonal=plane.clone();for(int i=12;i<50;i++)diagonal[i*w+i]+=7;
    diagnostic=new DepthOutlierFilter.Diagnostics();
    check(DepthOutlierFilter.mask(o,diagonal,gray,w,h,0,h,null,diagnostic,null).isEmpty(),"Connected diagonal wire including endpoints retained");
    check(diagnostic.counts[6]>0,"Directional safeguard recorded separately");
    float[] disconnected=plane.clone();int center=32*w+40;
    disconnected[center]+=5;disconnected[center-2]+=5;disconnected[center+2]+=5;
    check(DepthOutlierFilter.mask(o,disconnected,gray,w,h,0,h).get(center),"Disconnected collinear depths do not count as a connected feature");
    float[] uncertain=plane.clone();uncertain[center]+=2;
    float[] lr=new float[plane.length];Arrays.fill(lr,Float.NaN);lr[center]=2;
    check(DepthOutlierFilter.mask(o,uncertain,gray,w,h,0,h).get(center),"Well-conditioned surface departure rejected");
    diagnostic=new DepthOutlierFilter.Diagnostics();
    check(!DepthOutlierFilter.mask(o,uncertain,gray,w,h,0,h,lr,diagnostic,null).get(center),"Available stereo uncertainty widens the departure band");
    check(diagnostic.counts[10]>0,"Uncertainty-preserved departures have their own reason counter");
    float[] noisy=plane.clone();
    for(int y=0;y<h;y++)for(int x=0;x<w;x++)noisy[y*w+x]+=((x+y)%2==0?.4f:-.4f);
    noisy[center]=plane[center]+1.6f;
    check(!DepthOutlierFilter.mask(o,noisy,gray,w,h,0,h).get(center),"Measured local scatter protects uncertain departure");
  }

  private static void cameraVotes() {
    check(DepthOutlierFilter.cameraVote(10,9.99,10.01,9,.01,.005)==1,"Other camera confirms surface");
    check(DepthOutlierFilter.cameraVote(9.8,9.99,10.01,9,.01,.005)==2,"Other camera observes free space in front of surface");
    check(DepthOutlierFilter.cameraVote(10.2,9.99,10.01,9,.01,.005)==3,"Nearer occluder is neutral");
    check(DepthOutlierFilter.cameraVote(9.8,9.99,10.01,3,.01,.005)==0,"Sparse evidence is neutral");
    check(DepthOutlierFilter.cameraVote(9.8,9,11,9,.01,.005)==0,"Discontinuity is neutral");
    check(DepthOutlierFilter.cameraVote(9.8,9.99,10.01,9,.1,.005)==0,"Geometric uncertainty is neutral, never positive support");
    check(DepthOutlierFilter.cameraVote(6.6875734,6.982011,6.97,6.99,8,.0984,.004,.005)==0,
        "Recorded 4.4 percent halo mismatch cannot count as agreement");
    check(DepthOutlierFilter.cameraVote(10,0,9.99,10.01,8,.01,.005,.005)==0,
        "Missing center cannot become positive support through nearby samples");
    check(!DepthOutlierFilter.cameraReject(0,1) && !DepthOutlierFilter.cameraReject(2,2)
        && DepthOutlierFilter.cameraReject(1,2),"Two distinct cameras and majority required");
  }

  private static void patchContext() {
    int w=256,h=192;float[] z=new float[w*h];
    for(int y=0;y<h;y++)for(int x=0;x<w;x++)z[y*w+x]=(float)(1/(.1+.00008*x-.00005*y));
    var context=new SurfacePatchGrid(w,h,p->z[p],.005);
    check(context.predict(130,95,z[95*w+130]).kind()==1,"Multiscale inverse-depth plane retains a sloped surface");
    float[] wrong=z.clone();for(int y=88;y<104;y++)for(int x=120;x<136;x++)wrong[y*w+x]*=.95f;
    context=new SurfacePatchGrid(w,h,p->wrong[p],.005);
    check(context.predict(128,96,wrong[96*w+128]).kind()==2,"Annular context detects a coherent displaced patch");
    float[] missing=z.clone();for(int y=93;y<100;y++)for(int x=125;x<132;x++)missing[y*w+x]=0;
    context=new SurfacePatchGrid(w,h,p->missing[p],.005);
    check(context.predict(128,96,z[96*w+128]*.95).kind()==2,"Measured surrounding surface challenges a hole without inventing direct support");
    float[] edge=z.clone();for(int y=0;y<h;y++)for(int x=w/2;x<w;x++)edge[y*w+x]*=.7f;
    context=new SurfacePatchGrid(w,h,p->edge[p],.005);
    for(int x=122;x<135;x++)check(context.predict(x,96,edge[96*w+x]).kind()!=2,"Separate foreground/background planes preserve the depth edge");
    float[] wire=z.clone();for(int x=60;x<196;x++)wire[96*w+x]*=.8f;
    context=new SurfacePatchGrid(w,h,p->wire[p],.005);
    check(context.predict(128,96,wire[96*w+128]).kind()==0,"Connected thin structure remains protected in broad context");
    float[] sparse=new float[w*h];sparse[96*w+128]=7;
    context=new SurfacePatchGrid(w,h,p->sparse[p],.005);
    check(context.predict(128,96,7).kind()==0,"Sparse context does not supply a plane");
    float[] curved=new float[w*h];
    for(int y=0;y<h;y++)for(int x=0;x<w;x++)curved[y*w+x]=(float)(1/(.1-.000004*(x-128)*(x-128)));
    context=new SurfacePatchGrid(w,h,p->curved[p],.005);
    check(context.predict(128,96,curved[96*w+128]).kind()!=2,"Scale-dependent curvature cannot become a planar outlier");
  }

  /** Analytical camera raster evidence exercises the actual reference-fusion camera check. */
  private static void cameraEvidence(Path root)throws Exception {
    Options o=TestSupport.options();o.depthOutlierFilter="edge-aware";
    o.output=Files.createDirectories(root.resolve("camera-evidence"));o.memoryMb=128;
    Camera[] cameras=new Camera[6];int[] widths=new int[6],heights=new int[6];
    Arrays.fill(widths,64);Arrays.fill(heights,64);
    for(int i=0;i<6;i++) {
      cameras[i]=new Camera(2000,2000,31.5,31.5);
      cameras[i].translation=new double[]{-(i-2.5)*.002,0,0};
    }
    // Broad baselines for auxiliary pairs make their depth sufficiently certain.
    cameras[4].translation[0]=-1;cameras[5].translation[0]=1;
    try(var fusion=new DepthFusion(cameras,widths,heights,o)) {
      fusion.addPair(0,1);
      int[][] pairs={{2,4},{2,5},{3,4},{3,5}};
      for(int[] pair:pairs) {
        int id=fusion.addPair(pair[0],pair[1]);
        Camera c=cameras[pair[0]];
        for(int y=0;y<64;y++)for(int x=0;x<64;x++) {
          double z=10;double[] center=c.center();
          fusion.add(id,new double[]{center[0]+(x-31.5)*z/2000,(y-31.5)*z/2000,z});
        }
      }
      var method=DepthFusion.class.getDeclaredMethod("edgeVisibility",int.class,DepthFusion.Result.class);
      method.setAccessible(true);
      check((boolean)method.invoke(fusion,0,new DepthFusion.Result(new double[]{0,0,9.7},3,2,.0001,0,"accepted")),
          "Two independent auxiliary cameras reject a floating surface in actual paged evidence");
      check(!(boolean)method.invoke(fusion,0,new DepthFusion.Result(new double[]{0,0,10},3,2,.0001,0,"accepted")),
          "Actual multi-camera evidence confirms the true plane");
      check(!(boolean)method.invoke(fusion,0,new DepthFusion.Result(new double[]{0,0,10.3},3,2,.0001,0,"accepted")),
          "Occluded candidate is not rejected as free space");
    }
  }

  private static void imageCameraEvidence(Path root)throws Exception {
    long[] emitted=new long[2];
    for(int mode=0;mode<2;mode++) {
      Options o=TestSupport.options();o.depthOutlierFilter=mode==0?"off":"edge-aware";
      o.output=Files.createDirectories(root.resolve("image-camera-"+mode));
      o.memoryMb=128;o.cores=1;o.minViews=2;o.fusionMode="image";o.fusionStrategy="graph-average";
      o.fusionSpikeFilter=false;o.fusionDepthOutliers=false;o.speckleSize=0;o.fusionFreeSpace="off";
      Camera[] cameras=new Camera[6];int[] sizes=new int[6];Arrays.fill(sizes,64);
      double[] positions={-.005,2,-.001,.001,-1,1};
      for(int i=0;i<6;i++) {
        cameras[i]=new Camera(2000,2000,31.5,31.5);
        cameras[i].translation=new double[]{-positions[i],0,0};
      }
      try(var fusion=new DepthFusion(cameras,sizes,sizes,o)) {
        int[][] pairs={{0,1},{2,4},{2,5},{3,4},{3,5}};
        long[] starts=new long[pairs.length+1];long count=0;
        Path file=o.output.resolve("candidates.bin");
        try(var out=new java.io.DataOutputStream(Files.newOutputStream(file))) {
          for(int id=0;id<pairs.length;id++) {
            starts[id]=count;fusion.addPair(pairs[id][0],pairs[id][1]);
            for(int y=0;y<(id==0?1:64);y++)for(int x=0;x<(id==0?1:64);x++) {
              double[] world=id==0?new double[]{0,0,9.88}:
                  new double[]{positions[pairs[id][0]]+(x-31.5)*10/2000,(y-31.5)*10/2000,10};
              for(double v:world)out.writeDouble(v);
              out.write(new byte[]{10,20,30});out.writeInt(id);out.writeInt(y*64+x);
              for(int k=0;k<5;k++)out.writeFloat(.9f);
              fusion.add(id,world);count++;
            }
          }
        }
        starts[pairs.length]=count;
        try(var output=new FusionOutput(o,fusion,null,null)) {
          var counters=ImageFusion.run(o,fusion,file,starts,output,new TwoViewConfidence.Diagnostics(o));
          output.finish();emitted[mode]=counters.emitted;
          check(mode==0?counters.edgeFreeSpace==0:counters.edgeFreeSpace>0,
              "Image fusion activates uncertainty-aware independent-camera free-space checks only on opt-in");
          if(mode==1)check(counters.edgeSupported>0,"Image fusion records true-surface camera support");
        }
      }
    }
    check(emitted[1]<emitted[0],"Image camera evidence removes the floater while retaining the true plane");
  }

  /** No extra cameras: retain ordinary planar coverage, reject only the context suspect.
   * The same global precise-support gate would incorrectly remove the whole fixture. */
  private static void targetedPatchEvidence(Path root)throws Exception {
    long[] low=new long[2],high=new long[2],good=new long[2];
    for(int mode=0;mode<2;mode++) {
      Options o=TestSupport.options();o.depthOutlierFilter=mode==0?"off":"edge-aware";
      o.output=Files.createDirectories(root.resolve("targeted-patch-"+mode));
      o.memoryMb=128;o.cores=2;o.minViews=2;o.fusionMode="image";o.fusionStrategy="graph-average";
      o.fusionSpikeFilter=false;o.fusionDepthOutliers=false;o.speckleSize=0;o.fusionFreeSpace="off";
      Camera[] cameras={new Camera(180,180,79.5,63.5),new Camera(180,180,79.5,63.5)};
      cameras[1].translation[0]=-1;int[] widths={160,160},heights={128,128};
      try(var fusion=new DepthFusion(cameras,widths,heights,o)) {
        fusion.addPair(0,1);Path file=o.output.resolve("candidates.bin");long count=0;
        try(var out=new java.io.DataOutputStream(Files.newOutputStream(file))) {
          for(int y=0;y<128;y++)for(int x=0;x<160;x++) {
            double z=x>=72&&x<88&&y>=56&&y<72?9.5:10;
            if(x>=40&&x<56&&y>=56&&y<72)z=10.5;
            double[] world={(x-79.5)*z/180,(y-63.5)*z/180,z};
            for(double v:world)out.writeDouble(v);out.write(new byte[]{20,30,40});out.writeInt(0);out.writeInt(y*160+x);
            for(int k=0;k<5;k++)out.writeFloat(.9f);fusion.add(0,world);count++;
          }
        }
        try(var output=new FusionOutput(o,fusion,null,null)) {
          var counters=ImageFusion.run(o,fusion,file,new long[]{0,count},output,new TwoViewConfidence.Diagnostics(o));
          output.finish();if(mode==1)check(counters.patchUnsupported>0,"Suspect patch lacks precise independent support and is rejected");
        }
        try(var reader=new PointRecords.Reader(o.output.resolve("07_results/point-confidence.bin"))) {
          for(PointRecords.Row row;(row=reader.next())!=null;){if(row.sample().point[2]<9.75)low[mode]++;else if(row.sample().point[2]>10.25)high[mode]++;else good[mode]++;}
        }
      }
    }
    check(low[0]>100&&low[1]<low[0]/4,"Coherent displaced patch removed before averaging");
    check(high[0]>100&&high[1]<high[0]/4,"Coherent farther patch also requires precise support");
    check(good[1]>=good[0]*.95,"Normal surface coverage survives without a global extra-camera gate");
  }

  private static void referencePatchEvidence(Path root)throws Exception {
    Options o=TestSupport.options();o.depthOutlierFilter="edge-aware";
    o.output=Files.createDirectories(root.resolve("reference-patch"));o.memoryMb=128;
    Camera[] cameras=new Camera[6];int[] w=new int[6],h=new int[6];Arrays.fill(w,160);Arrays.fill(h,128);
    double[] position={-.01,1,-.01,.01,-.5,.5};
    for(int i=0;i<6;i++){cameras[i]=new Camera(500,500,79.5,63.5);cameras[i].translation[0]=-position[i];}
    try(var fusion=new DepthFusion(cameras,w,h,o)) {
      fusion.addPair(0,1);
      for(int[] pair:new int[][]{{2,4},{2,5},{3,4},{3,5}}) {
        int id=fusion.addPair(pair[0],pair[1]);
        for(int y=0;y<128;y++)for(int x=0;x<160;x++) {
          if(Math.abs(x-80)<=4&&Math.abs(y-64)<=4)continue;
          fusion.add(id,new double[]{position[pair[0]]+(x-79.5)*10/500,(y-63.5)*10/500,10});
        }
      }
      fusion.prepare(1);
      var method=DepthFusion.class.getDeclaredMethod("edgeVisibility",int.class,DepthFusion.Result.class);method.setAccessible(true);
      check((boolean)method.invoke(fusion,0,new DepthFusion.Result(new double[]{0,0,9.7},3,2,.0001,0,"accepted")),
          "Paged multi-camera surrounding planes challenge a floating point at sparse center patches");
      check(!(boolean)method.invoke(fusion,0,new DepthFusion.Result(new double[]{0,0,10},3,2,.0001,0,"accepted")),
          "Sparse center data does not globally remove the true surface");
    }
  }

  private static void windowed(Path root,Options source,float[] values,byte[] gray,
      BitSet expected,int w,int h)throws Exception {
    Options o=source.copy();o.output=Files.createDirectories(root.resolve("windowed"));
    o.stereoReportDirectory=o.output;o.memoryMb=256;o.disparities=16;o.pyramidLevels=1;
    try(var store=new WindowedRasters(o)) {
      var left=store.new Image(w,h);var right=store.new Image(w,h);
      for(int i=0;i<values.length;i++)left.gray.set(i,gray[i]&255);
      try(var stereo=new WindowedStereo(store,left,right,o)) {
        for(int i=0;i<values.length;i++)stereo.put(i,WindowedStereo.FINAL,values[i]);
        stereo.rows=1;
        int files=store.files.size();
        var filter=WindowedStereo.class.getDeclaredMethod("depthOutliers");filter.setAccessible(true);filter.invoke(stereo);
        for(int i=0;i<values.length;i++)check(expected.get(i)?Float.isNaN(stereo.get(i,WindowedStereo.FINAL)):
            Float.floatToRawIntBits(values[i])==Float.floatToRawIntBits(stereo.get(i,WindowedStereo.FINAL)),
            "Actual disk-backed windowed filter matches immutable full-raster result");
        check(stereo.stats.depthOutlierRejected==expected.cardinality(),"Windowed rejection counter");
        check(store.files.size()==files,"Windowed rejection bitmap released");
      }
    }
  }

  private static void defaults(Path root)throws Exception {
    Options o=TestSupport.options();
    check(o.depthOutlierFilter.equals("off"),"Opt-in disabled by default");
    float[] values={Float.intBitsToFloat(0x7fc01234),-0.0f,1,5};float[] before=values.clone();
    var unchanged=new CensusSgm.Result(2,2,values);
    check(DepthOutlierFilter.filter(o,unchanged,new byte[4],null)==unchanged,"Disabled path returns same object");
    for(int i=0;i<values.length;i++)check(Float.floatToRawIntBits(before[i])==Float.floatToRawIntBits(values[i]),"Disabled NaN payload and signed zero unchanged");
    Options parsed=Options.parse(new String[]{"-i",root.toString(),"--depth-outlier-filter","edge-aware"});
    check(parsed.depthOutlierFilter.equals("edge-aware"),"CLI opt-in accepted");
    for(String[] bad:List.of(new String[]{"--depth-outlier-filter","invalid"},
        new String[]{"--depth-outlier-radius","1"},new String[]{"--depth-outlier-deviation","0"},
        new String[]{"--depth-outlier-edge-threshold","256"},
        new String[]{"--depth-outlier-filter","edge-aware","--dense-method","patchmatch"})) {
      var args=new ArrayList<>(List.of("-i",root.toString()));args.addAll(List.of(bad));boolean rejected=false;
      try{Options.parse(args.toArray(String[]::new));}catch(IllegalArgumentException e){rejected=true;}
      check(rejected,"Invalid/unsupported settings rejected "+Arrays.toString(bad));
    }
    int w=80,h=48;byte[] left=new byte[w*h],right=new byte[w*h];new Random(55).nextBytes(left);
    for(int y=0;y<h;y++)for(int x=0;x<w-8;x++)right[y*w+x]=left[y*w+x+8];
    o.minDisparity=2;o.disparities=16;o.speckleSize=0;o.memoryMb=256;
    var original=new CensusSgm(o).compute(left,right,w,h);
    var omitted=DenseMatcher.compute(o,left,right,null,null,w,h,null,null,null);
    Options explicit=o.copy();explicit.depthOutlierFilter="off";
    var disabled=DenseMatcher.compute(explicit,left,right,null,null,w,h,null,null,null);
    for(int i=0;i<left.length;i++) {
      check(Float.floatToRawIntBits(original.disparity[i])==Float.floatToRawIntBits(omitted.disparity[i]),"Omitted filter matches original SGM bitwise");
      check(Float.floatToRawIntBits(disabled.disparity[i])==Float.floatToRawIntBits(omitted.disparity[i]),"Explicit off matches omitted bitwise");
    }
    o.depthOutlierFilter="edge-aware";o.stereoReportDirectory=root.resolve("report");
    var filtered=DenseMatcher.compute(o,left,right,null,null,w,h,null,null,null);
    for(int i=0;i<left.length;i++)check(Float.isNaN(filtered.disparity[i])
        || Float.floatToRawIntBits(filtered.disparity[i])==Float.floatToRawIntBits(omitted.disparity[i]),"Enabled path only removes points");
    check(Files.isRegularFile(o.stereoReportDirectory.resolve("depth_outlier_filter.json")),"Rejection report written");
    // Filter settings must participate in stereo cache keys; cached results retain counters.
    o.cacheDirectory=Files.createDirectories(root.resolve("cache"));
    var first=ReuseCache.stereo(o,left,right,null,null,w,h,null,null,null,null);
    var again=ReuseCache.stereo(o,left,right,null,null,w,h,null,null,null,null);
    check(first.stats.depthOutlierRejected==again.stats.depthOutlierRejected,"Cache round-trip rejection counter");
    check(Arrays.equals(first.stats.depthOutlierReasons,again.stats.depthOutlierReasons),"Cache preserves decision reasons");
    long enabledEntries;
    try(var files=Files.walk(o.cacheDirectory)){enabledEntries=files.filter(p->p.getFileName().toString().equals("payload.bin")).count();}
    o.depthOutlierFilter="off";
    var off=ReuseCache.stereo(o,left,right,null,null,w,h,null,null,null,null);
    try(var files=Files.walk(o.cacheDirectory)) {
      check(files.filter(p->p.getFileName().toString().equals("payload.bin")).count()==enabledEntries+1,
          "Enabled and disabled modes have separate cache entries even when outputs coincide");
    }
    for(int i=0;i<left.length;i++)check(Float.floatToRawIntBits(off.disparity[i])==Float.floatToRawIntBits(omitted.disparity[i]),"Cache never substitutes enabled results for disabled mode");
  }
}
