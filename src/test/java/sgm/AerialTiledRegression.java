package sgm;

import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;
import org.opencv.core.*;

/** Independent signed-disparity, crop-origin, seam, bounded-fusion and opt-in checks. Requires GPU. */
public final class AerialTiledRegression {
  private static int checks;
  private static void check(boolean ok,String label) {
    checks++;if(!ok)throw new AssertionError(label);
  }
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();Core.setNumThreads(2);
    Path root=Files.createTempDirectory(Path.of("target"),"aerial-tiled-regression-");
    cli(root);
    for(int disparity:new int[]{18,-18})stereo(root,disparity);
    subdivisionAndFailure(root);
    integration(root,false);
    integration(root,true);
    GpuSgm.closeSession();
    System.out.println("PASS aerial tiled checks="+checks+"; fixtures="+root);
  }
  private static Options options(Path output) throws Exception {
    Options o=new Options();o.output=Files.createDirectories(output);o.projectDirectory=o.output;
    o.densePipeline="aerial-tiled";o.compute="gpu";o.fusionCompute="cpu";
    o.surfaceConsolidation="none";o.pointAttributes="none";o.fusionEvidence="pair";
    o.rectified=true;o.baseline=1;o.minViews=2;o.cores=2;o.memoryMb=1024;o.gpuMemoryMb=512;
    o.aerialTileSize=96;o.aerialOverlap=16;o.aerialCoarseDimension=128;
    o.aerialDisparityMargin=4;o.minDisparity=8;o.disparities=33;o.disparitiesExplicit=true;
    o.maxDimension=2048;o.step=1;o.speckleSize=0;o.correlation=.4;o.twoViewConfidenceThreshold=.1;
    o.fusionCacheMb=8;o.fusionBatchSize=256;o.depthRefinementMethod="weighted";
    return o;
  }
  private static void cli(Path root) throws Exception {
    check(new Options().densePipeline.equals("standard"),"Existing default is standard");
    Options parsed=Options.parse(new String[]{"-i",root.toString(),"--dense-pipeline","aerial-tiled",
        "--surface-consolidation","none"});
    check(parsed.densePipeline.equals("aerial-tiled"),"Explicit opt-in accepted");
    for(String[] extra:List.of(new String[]{"--compute","cpu"},
        new String[]{"--fusion-mode","image"},new String[]{"--fusion-strategy","graph-average"},
        new String[]{"--aerial-tile-size","32"})) {
      var args=new ArrayList<>(List.of("-i",root.toString(),"--dense-pipeline","aerial-tiled",
          "--surface-consolidation","none"));args.addAll(List.of(extra));
      boolean rejected=false;
      try{Options.parse(args.toArray(String[]::new));}catch(IllegalArgumentException e){rejected=true;}
      check(rejected,"Unsupported aerial combination rejected: "+Arrays.toString(extra));
    }
  }
  private static byte[] texture(int w,int h) {
    byte[] data=new byte[w*h];new Random(314159).nextBytes(data);return data;
  }
  private static AerialTileStereo.Rectifier synthetic(byte[] left,byte[] right,int w,int h) {
    return (reverse,x,y,tw,th,scale)->{
      byte[] gray=new byte[tw*th],color=new byte[3*tw*th],mask=new byte[tw*th];
      byte[] input=reverse?right:left;
      for(int yy=0;yy<th;yy++)for(int xx=0;xx<tw;xx++) {
        int gx=(int)Math.round(x+xx*scale),gy=(int)Math.round(y+yy*scale),i=yy*tw+xx;
        if(gx<0 || gy<0 || gx>=w || gy>=h)continue;
        gray[i]=input[gy*w+gx];mask[i]=(byte)255;
        Arrays.fill(color,3*i,3*i+3,gray[i]);
      }
      return new AerialTileStereo.Pixels(tw,th,gray,color,mask);
    };
  }
  private static void stereo(Path root,int d) throws Exception {
    int w=288,h=192;byte[] left=texture(w,h),right=new byte[w*h];
    for(int y=0;y<h;y++)for(int x=0;x<w;x++)if(x+d>=0 && x+d<w)right[y*w+x]=left[y*w+x+d];
    Options o=options(root.resolve("signed-"+d));o.minDisparity=d-12;o.disparities=25;
    float[] actual=new float[w*h];Arrays.fill(actual,Float.NaN);
    var stereo=new AerialTileStereo(o,synthetic(left,right,w,h),w,h);
    stereo.run(new AerialTileStereo.Sink(){
      public double seamError(AerialTileStereo.Match m){return 0;}
      public void accept(AerialTileStereo.Match m,boolean reject){
        var c=m.prepared().plan().core();
        for(int y=c.y();y<c.y()+c.height();y++)for(int x=c.x();x<c.x()+c.width();x++)actual[y*w+x]=m.disparity(x,y);
      }
    });
    int valid=0,bad=0,seamValid=0;
    for(int y=24;y<h-24;y++)for(int x=40;x<w-40;x++) {
      float value=actual[y*w+x];if(!Float.isFinite(value))continue;
      valid++;if(Math.abs(value-d)>.5)bad++;
      if(x%96<3 || y%96<3)seamValid++;
    }
    check(valid>20000,"Signed "+d+" preserves dense coverage: "+valid);
    check(bad<valid/100,"Signed "+d+" disparity/crop offsets: bad="+bad);
    check(seamValid>500,"Valid depths cross tile seams");
    check(stereo.peakEstimatedGpuBytes<512L*1048576,"GPU working set bounded by budget");
  }
  private static void integration(Path root,boolean filter) throws Exception {
    int w=288,h=192,d=18;byte[] left=texture(w,h),right=new byte[w*h];
    for(int y=0;y<h;y++)for(int x=0;x<w-d;x++)right[y*w+x]=left[y*w+x+d];
    Path images=Files.createDirectories(root.resolve("images"));
    for(int side=0;side<2;side++) {
      byte[] data=side==0?left:right;BufferedImage image=new BufferedImage(w,h,BufferedImage.TYPE_INT_RGB);
      for(int y=0;y<h;y++)for(int x=0;x<w;x++){int v=data[y*w+x]&255;image.setRGB(x,y,v*0x010101);}
      check(ImageIO.write(image,"TIFF",images.resolve(side+".tif").toFile()),"TIFF fixture written");
    }
    Options o=options(root.resolve(filter?"integration-filter":"integration"));
    if(filter)o.depthOutlierFilter="edge-aware";
    Calibration calibration=new Calibration(180,180,(w-1)*.5,(h-1)*.5,new double[5],w,h);
    var frames=new ArrayList<ImageFrame>();
    try {
      for(int i=0;i<2;i++) {
        var f=new ImageFrame(i,images.resolve(i+".tif"),calibration,o,true);
        f.camera.translation[0]=-i;f.camera.registered=true;frames.add(f);
      }
      var r=new DenseReconstruction.Rectified(frames.get(0),frames.get(1));
      r.r1=Math3.identity();r.r2=Math3.identity();
      r.p1=new double[]{180,0,(w-1)*.5,0,0,180,(h-1)*.5,0,0,0,1,0};r.p2=r.p1.clone();r.p2[3]=-180;
      var reader=new AerialRectifier(r,o);
      var region=reader.read(false,31,29,80,60,1);
      for(int y=5;y<55;y++)for(int x=5;x<75;x++)
        check(region.gray()[y*80+x]==left[(y+29)*w+x+31],"Rectified source-region origin");
      r.vertical=true;
      var vertical=reader.read(false,31,29,80,60,1);
      for(int y=5;y<55;y++)for(int x=5;x<75;x++)
        check(vertical.gray()[y*80+x]==left[(x+31)*w+y+29],"Vertical epipolar source-region transpose");
      r.vertical=false;
      var dense=new DenseReconstruction(o,frames,null,null,List.of());
      long points=dense.run(List.of(new FeatureTracks.Pair(0,1)));
      check(points>20000,"End-to-end block fusion retains plane: "+points);
      check(Files.isRegularFile(o.output.resolve("dense.las")),"LAS published");
      if(filter)check(Files.readString(o.output.resolve("06_fusion/backend.json")).contains("edge_aware_camera_checks"),
          "Aerial camera-evidence diagnostics published");
      // Independent LAS decode: the known 180px focal, 1-unit baseline, 18px
      // disparity plane must remain at Z=10 after local/global crop conversion.
      var las=java.nio.ByteBuffer.wrap(Files.readAllBytes(o.output.resolve("dense.las")))
          .order(java.nio.ByteOrder.LITTLE_ENDIAN);
      int offset=las.getInt(96),record=Short.toUnsignedInt(las.getShort(105));
      double scaleZ=las.getDouble(147),offsetZ=las.getDouble(171);
      double[] depths=new double[(int)points];
      for(int i=0;i<depths.length;i++)depths[i]=las.getInt(offset+i*record+8)*scaleZ+offsetZ;
      Arrays.sort(depths);
      check(Math.abs(depths[depths.length/2]-10)<.05,"Triangulated plane median height");
      check(depths[(int)(depths.length*.95)]-depths[(int)(depths.length*.05)]<.15,
          "No tile-induced bending of the synthetic plane");
      check(Files.readString(o.output.resolve("06_fusion/aerial_tiled.json")).contains("\"candidate_spool_bytes\":0"),"No candidate spool");
      try(var files=Files.walk(o.output)) {
        check(files.noneMatch(p->p.getFileName().toString().endsWith(".tiles")),"Compact and evidence temporary files cleaned");
      }
    } finally {for(var f:frames)f.close();}
  }
  private static void subdivisionAndFailure(Path root) throws Exception {
    int w=192,h=128,d=18;byte[] left=texture(w,h),right=new byte[w*h];
    for(int y=0;y<h;y++)for(int x=0;x<w-d;x++)right[y*w+x]=left[y*w+x+d];
    var base=synthetic(left,right,w,h);
    Options o=options(root.resolve("subdivision"));o.minDisparity=6;o.disparities=25;
    AerialTileStereo.Rectifier bounded=(reverse,x,y,tw,th,scale)->{
      if(scale==1 && tw>160)throw new AerialTileStereo.TileTooLarge("Injected large source footprint");
      return base.read(reverse,x,y,tw,th,scale);
    };
    byte[] ownership=new byte[w*h];
    var stereo=new AerialTileStereo(o,bounded,w,h);
    stereo.run(new AerialTileStereo.Sink(){
      public double seamError(AerialTileStereo.Match m){return m.prepared().plan().full()?0:1;}
      public void accept(AerialTileStereo.Match m,boolean reject){
        var c=m.prepared().plan().core();
        for(int y=c.y();y<c.y()+c.height();y++)for(int x=c.x();x<c.x()+c.width();x++)ownership[y*w+x]++;
      }
    });
    check(stereo.subdivisions>0,"Large footprints subdivide rather than allocate/spill");
    check(stereo.retries>0,"Seam disagreement triggers restored pair interval");
    for(byte value:ownership)check(value==1,"Subdivision/retry commits each core pixel once");
    boolean failed=false;
    try {
      new AerialTileStereo(o,base,w,h).run(new AerialTileStereo.Sink(){
        public double seamError(AerialTileStereo.Match m){return 0;}
        public void accept(AerialTileStereo.Match m,boolean reject)throws IOException{
          throw new IOException("Injected output failure");
        }
      });
    } catch(IOException e){failed=e.getMessage().contains("Injected output failure");}
    check(failed,"Sink failure propagates");
    check(Thread.getAllStackTraces().keySet().stream().noneMatch(t->t.isAlive()
        && t.getName().equals("aerial-rectification")),"Decoder worker closes on failure");
    Options unavailable=o.copy();unavailable.gpuDevice=99999;
    boolean rejected=false;
    try{new AerialTileStereo(unavailable,base,w,h);}catch(RuntimeException e){rejected=true;}
    check(rejected,"No silent CPU fallback when GPU selection fails");
  }
}
