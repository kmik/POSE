package sgm;

import java.awt.event.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import javax.swing.*;

/** Viewer geometry, source identity, culling, cancellation and asynchronous publication controls. */
final class CloudViewerRegression {
  static int checks;
  static void check(boolean ok,String message){checks++;if(!ok)throw new AssertionError(message);}
  static void near(double a,double b,String message){check(Math.abs(a-b)<1e-7,message+": "+a+" vs "+b);}
  public static void main(String[] args) throws Exception {
    Path root=Files.createTempDirectory(Path.of("target"),"viewer-");
    if(args.length>0 && args[0].equals("preview")){preview(root);return;}
    System.out.println("PASS viewer: "+run(root)+" checks; fixtures: "+root);
  }
  static int run(Path root) throws Exception {
    checks=0;Files.createDirectories(root);geometry();display();spatial();detailCache();source(root);canvas();return checks;
  }

  static void geometry() {
    var initial=new CloudRenderer.View(new double[]{0,0,0},0,55,100,45,false);
    near(initial.orbit(0,180).pitch(),235,"pitch crosses both poles");
    near(initial.orbit(720,720).pitch(),55,"complete pitch revolutions");
    near(initial.orbit(725,0).yaw(),5,"complete Z revolutions");
    near(initial.withRoll(725).roll(),5,"complete roll revolutions");
    for(int yaw=0;yaw<360;yaw+=45)for(int pitch=0;pitch<360;pitch+=45)for(int roll=0;roll<360;roll+=45) {
      var v=new CloudRenderer.View(new double[]{10,20,30},yaw,pitch,100,45,false,roll);
      double[] r=v.right(),u=v.up(),f=v.forward();
      near(dot(r,u),0,"basis right/up orthogonal");near(dot(r,f),0,"basis right/forward orthogonal");near(dot(u,f),0,"basis up/forward orthogonal");
      near(dot(r,r),1,"unit right");near(dot(u,u),1,"unit up");
      for(boolean ortho:new boolean[]{true,false}) {
        var pr=new CloudRenderer.Projector(v.withOrtho(ortho),800,600);
        double[] p=pr.unproject(310,230,90),projected=new double[3];
        check(pr.project(p[0],p[1],p[2],projected),"unprojected point in front");
        near(projected[0],310,"projection X round trip");near(projected[1],230,"projection Y round trip");near(projected[2],90,"projection depth round trip");
      }
    }
    double[] target={1,2,3};var immutable=new CloudRenderer.View(target,0,0,1,45,false);target[0]=99;near(immutable.target()[0],1,"input target copied");
    target=immutable.target();target[0]=99;near(immutable.target()[0],1,"target accessor copied");
    CloudData a=new CloudData(),b=new CloudData();a.origin[0]=600000;b.origin[0]=600100;a.origin[2]=30;b.origin[2]=90;
    var translated=CloudViewerControls.translate(initial,a,b);near(translated.target()[0],-100,"comparison world X");near(translated.target()[2],-60,"comparison world height");
    check(CloudViewerControls.compatible(a,b),"same coordinate declarations compatible");a.crs="EPSG:3067";check(!CloudViewerControls.compatible(a,b),"unknown CRS cannot be paired with declared CRS");
    b.crs=a.crs;b.multiplier=100;check(!CloudViewerControls.compatible(a,b),"different output units refused");
  }
  static double dot(double[] a,double[] b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
  static CloudData points(int count) {
    CloudData d=new CloudData();d.n=count;d.total=count;d.x=new float[count];d.y=new float[count];d.z=new float[count];d.rgb=new int[count];d.cls=new byte[count];d.originalIds=new long[count];
    for(int i=0;i<count;i++){d.x[i]=i%100;d.y[i]=i/100;d.rgb[i]=0x336699;d.originalIds[i]=i;}
    return d;
  }
  static void display() {
    CloudData d=points(3);d.x=new float[]{0,0,1};d.y=new float[]{0,0,0};d.z=new float[]{0,10,0};d.cls=new byte[]{2,5,6};
    d.attributes.add(new CloudData.Attribute("confidence","Confidence",new float[]{.5f,.8f,Float.NaN}));
    var style=new CloudDisplay(new CloudDisplay.Box(-1,-1,-1,1,1,10),Map.of("confidence",new CloudDisplay.Range(.5,.8)),Set.of(2,5),null,0xff9800,12);
    check(style.dense(d,0),"inclusive lower confidence and class");check(style.dense(d,1),"inclusive upper confidence bound at stored float precision");
    check(!style.dense(d,2),"unavailable confidence rejected");
    style=new CloudDisplay(null,Map.of("unknown",new CloudDisplay.Range(0,1)),Set.of(),null,0,0);check(!style.dense(d,0),"missing attribute rejected");
    var top=new CloudRenderer.View(new double[3],0,90,100,45,true);
    var q=new CloudRenderer.Request(d,top,201,201,CloudRenderer.Colour.RGB,-1,1,false,true,false,1,0,1);
    var f=CloudRenderer.render(q);check(f.pick(100,100,0)==1,"nearest depth at overlapping pixel");
    var clip=new CloudDisplay(new CloudDisplay.Box(-1,-1,-1,2,1,1),Map.of(),Set.of(),0xff0000,0xff9800,0);
    q=new CloudRenderer.Request(d,top,201,201,CloudRenderer.Colour.RGB,-1,1,false,true,false,1,0,1,clip,()->false,1000);
    f=CloudRenderer.render(q);check(f.pick(100,100,0)==0,"clip excludes higher layer");check((f.argb()[100*201+100]&0xffffff)==0xff0000,"dense tint applied");
    long[] buffer=new long[9];Arrays.fill(buffer,CloudRenderer.EMPTY);buffer[4]=(long)Float.floatToIntBits(100)<<32|7;buffer[3]=(long)Float.floatToIntBits(1)<<32|8;
    var manual=new CloudRenderer.Frame(3,3,new int[9],buffer,q);check(manual.pick(1,1,1)==7,"screen-nearest point before depth at adjacent pixel");
    Arrays.fill(buffer,CloudRenderer.EMPTY);buffer[0]=1;check(new CloudRenderer.Frame(3,3,new int[9],buffer,q).pick(1,1,1)==-1,"pick radius circular");
    boolean cancelled=false;try{CloudRenderer.render(new CloudRenderer.Request(d,top,201,201,CloudRenderer.Colour.RGB,-1,1,false,true,false,1,0,1,clip,()->true,1000));}catch(CancellationException expected){cancelled=true;}
    check(cancelled,"cancelled render does not produce a frame");
    near(CloudRenderer.normalize(.8f,.8f,.8f),.5,"constant scalar values use middle colour");
    CloudData backdrop=points(1);backdrop.x[0]=20;backdrop.y[0]=0;
    var withOverview=CloudRenderer.render(new CloudRenderer.Request(d,top,201,201,CloudRenderer.Colour.RGB,-1,1,false,true,false,1,0,1,CloudDisplay.DEFAULT,()->false,1000,backdrop));
    double[] screen=new double[3];new CloudRenderer.Projector(top,201,201).project(20,0,0,screen);
    check((withOverview.pick((int)screen[0],(int)screen[1],1)&CloudRenderer.OVERVIEW)!=0,"coarse context keeps distinct pick identity");
    long[] reuse=new long[201*201];int[] pixels=new int[reuse.length];var reused=CloudRenderer.render(q,reuse,pixels);
    check(reused.buffer()==reuse && reused.argb()==pixels,"renderer reuses caller buffers");
  }
  static void spatial() {
    CloudData d=points(100_000);d.spatial=new CloudSpatialIndex(d);
    var top=new CloudRenderer.View(new double[]{50,500,0},0,90,1500,45,true);
    var pr=new CloudRenderer.Projector(top,1200,1200);
    int[] ids=d.spatial.select(pr,null,2048,()->false);check(ids.length<=2048 && ids.length>0,"spatial LOD respects preview cap");
    BitSet seen=new BitSet();for(int id:ids){check(!seen.get(id),"LOD point identity unique");seen.set(id);}
    var outside=new CloudDisplay.Box(1e6,1e6,1e6,1e6+1,1e6+1,1e6+1);
    check(d.spatial.select(pr,outside,2048,()->false).length==0,"spatial clip culls whole tree");
    int[] full=d.spatial.select(pr,null,d.n,()->false);check(full.length==d.n,"full visible tree can render every loaded point");
    check(!pr.visible(outside),"view frustum rejects remote node");
    check(pr.visible(new CloudDisplay.Box(0,0,0,100,1000,0)),"view frustum retains cloud bounds");
    check(CloudData.budget(12_000_000,64L<<20,8)<1_000_000,"memory budget caps point allocation");
  }
  static void detailCache() {
    CloudData overview=points(100_000);overview.total=1_000_000;
    overview.lasFiles.add(Path.of("unavailable-on-purpose.las"));
    CloudData detail=points(2000);detail.total=overview.total;detail.regionTotal=20_000;
    detail.lasFiles.addAll(overview.lasFiles);
    detail.detailRegion=new CloudDisplay.Box(-20,-20,-20,20,20,20);
    var cache=new CloudDetailCache();cache.accept(overview);
    var home=new CloudRenderer.View(new double[3],0,90,1000,45,true);
    cache.navigated(home);cache.navigated(home.zoom(.1));cache.accept(detail);
    check(cache.overview==overview && cache.overview.n==100_000,"local detail retains the original dense overview");
    overview.spatial=new CloudSpatialIndex(overview);
    var wide=new CloudRenderer.View(new double[]{50,500,0},0,90,1500,45,true);
    var returned=CloudRenderer.render(new CloudRenderer.Request(detail,wide,1200,1200,CloudRenderer.Colour.RGB,-1,1,false,true,false,1,0,1,CloudDisplay.DEFAULT,()->false,Integer.MAX_VALUE,overview));
    double[] screen=new double[3];new CloudRenderer.Projector(wide,1200,1200).project(99,999,0,screen);
    int outsidePoint=returned.pick((int)screen[0],(int)screen[1],2);
    check(outsidePoint>=0 && (outsidePoint&CloudRenderer.OVERVIEW)!=0,"first zoom-out frame includes points outside the old detail box");
    var outside=new CloudDisplay.Box(-100,-100,-100,100,100,100);
    for(int i=0;i<40;i++) {
      cache.navigated(home.zoom(.2+i*.01));
      check(cache.cached(outside,false)==overview,"zoom-out step restores resident points without opening the source file");
    }
    cache.navigated(home.zoom(.1));
    var inside=new CloudDisplay.Box(-15,-15,-15,15,15,15);
    check(cache.cached(inside,false)==detail,"returning to cached local detail needs no disk reload");
    var closer=new CloudDisplay.Box(-3,-3,-3,3,3,3);
    check(cache.cached(closer,false)==null,"substantially finer zoom requests finer evidence when sampling was incomplete");
    detail.regionTotal=detail.n;
    check(cache.cached(closer,false)==detail,"complete local evidence is reused even at finer zoom");
    cache.navigated(home.zoom(.2));cache.navigated(home.zoom(.2).at(new double[]{1000,0,0}));
    check(!cache.zoomingOut && cache.cached(outside,false)==null,"panning after zoom-out can request a new local region");
    var padded=CloudDetailCache.padded(inside);
    check(CloudViewerControls.encloses(padded,inside) && padded.x1()>inside.x1(),"detail requests include a reuse margin");
    near(CloudDetailCache.detailBudget(1_000_000),500_000,"retained overview and two detail buffers share the original memory plan");
    var replacement=points(10);replacement.lasFiles.add(Path.of("different-run.las"));cache.accept(replacement);
    check(cache.overview==replacement && cache.detail==null,"switching source files invalidates cached detail");
    cache.clear();check(cache.overview==null && cache.detail==null,"closing releases both cached datasets");
  }

  static void source(Path root) throws Exception {
    Path run=Files.createDirectories(root.resolve("runs/base")),result=Files.createDirectories(run.resolve("07_results"));
    int count=140_000;
    try(CloudWriter cloud=new CloudWriter(result,1,null,true);PointRecords.Writer records=new PointRecords.Writer(result.resolve("point-confidence.bin"))) {
      cloud.writePly(false);
      for(int i=0;i<count;i++) {
        double x=600000+i*.01,y=6900000+i%100*.01,z=30+i%50*.01;
        cloud.point(x,y,z,i%256,100,200,new double[]{0,0,1},new SurfaceAttributes(i*.001,.01,.9,1,20,0,false));
        var p=new SurfaceConsolidator.Sample();p.id=i;p.point=new double[]{x,y,z};p.views=2+i%5;p.supportingCameras.set(i%3);records.point(i,p,1,i/(double)count);
      }
    }
    CloudData overview=CloudData.load(run,100,null);check(overview.n==100,"overview budget");check(overview.sourceIndex.blocks.size()==3,"source record-block index");
    long modified=Files.getLastModifiedTime(result.resolve("dense.las")).toMillis(),size=Files.size(result.resolve("dense.las"));
    var region=new CloudDisplay.Box(600010-overview.origin[0],6900000-overview.origin[1],29-overview.origin[2],600010.99-overview.origin[0],6900002-overview.origin[1],31-overview.origin[2]);
    var metadata=new CloudPointMetadata(result.resolve("point-confidence.bin"));
    var evidence=metadata.find(130_000);near(evidence.confidence(),130_000/(double)count,"random-access provenance score");check(evidence.views()==2 && evidence.cameras().get(1),"random-access provenance views and cameras");
    near(metadata.find(1000).confidence(),1000/(double)count,"provenance bookmarks allow backwards seeks");
    near(metadata.find(139999).confidence(),139999/(double)count,"provenance bookmark forward seek");
    check(metadata.find(count)==null,"missing provenance ID");
    CloudData detail=CloudData.load(run,200,null,region,overview.sourceIndex);
    check(detail.n==100 && detail.regionTotal==100,"detail recovers all 100 local points despite sparse overview");
    check(CloudRenderer.home(detail).distance()>1000,"Home from detail fits full source extent");
    check(detail.sourceIndex==overview.sourceIndex,"source hierarchy reused");
    SwingUtilities.invokeAndWait(()->{
      var view=new CloudView();view.setSize(40,40);view.setData(overview,true);
      var request=new CloudRenderer.Request(overview,view.view,40,40,CloudRenderer.Colour.RGB,-1,1,false,true,false,1,0,1);
      long[] buffer=new long[1600];Arrays.fill(buffer,CloudRenderer.EMPTY);buffer[20*40+20]=(long)Float.floatToIntBits(100)<<32|99;
      var frame=new CloudRenderer.Frame(40,40,new int[1600],buffer,request);view.frameShown=frame;
      double[] before=view.pointAt(20,20);view.setData(detail,false);
      check(view.frameShown==frame,"switching resident detail keeps the last completed frame visible");
      check(Arrays.equals(before,view.pointAt(20,20)),"picking during a detail transition uses the displayed dataset");
      view.dispose();
    });
    check(detail.originalIds[0]==1000 && detail.originalIds[99]==1099,"detail retains exact source IDs");
    check(detail.loadConfidence(run,null),"detail confidence read");
    for(int i=0;i<detail.n;i++) {
      near(detail.attribute("confidence").values()[i],detail.pointId(i)/(double)count,"confidence maps to original ID");
      near(detail.attribute("roughness_rms").values()[i],detail.pointId(i)*.001,"attributes map to source record");
    }
    var limited=CloudData.load(run,25,null,region,overview.sourceIndex);check(limited.n==25 && limited.regionTotal==100,"local reservoir has exact point cap and population");
    for(int i=1;i<limited.n;i++)check(limited.pointId(i)>limited.pointId(i-1),"reservoir IDs sorted for provenance");
    var again=CloudData.load(run,25,null,region,overview.sourceIndex);check(Arrays.equals(limited.originalIds,again.originalIds),"local sample deterministic");
    check(limited.loadConfidence(run,null),"reservoir confidence read");for(int i=0;i<limited.n;i++)near(limited.attribute("confidence").values()[i],limited.pointId(i)/(double)count,"reservoir provenance");
    var empty=CloudData.load(run,25,null,new CloudDisplay.Box(1e9,1e9,1e9,1e9+1,1e9+1,1e9+1),overview.sourceIndex);check(empty.n==0,"empty region supported");
    check(Files.size(result.resolve("dense.las"))==size && Files.getLastModifiedTime(result.resolve("dense.las")).toMillis()==modified,"viewer leaves LAS unchanged");
    Thread.currentThread().interrupt();boolean cancelled=false;try{CloudData.load(run,25,null);}catch(InterruptedIOException expected){cancelled=true;}finally{Thread.interrupted();}check(cancelled,"source load interruption");
    Path tiles=Files.createDirectories(root.resolve("runs/tiles/07_results"));Files.copy(result.resolve("dense.las"),tiles.resolve("tile_1.las"));Files.copy(result.resolve("dense.las"),tiles.resolve("tile_2.las"));
    var tiled=CloudData.load(tiles.getParent(),200,null);check(tiled.total==2L*count && tiled.n==200,"LAS tiles share global sample budget");check(tiled.originalIds[100]>=count,"LAS tile source IDs global");
  }
  static void canvas() throws Exception {
    CloudView[] holder=new CloudView[1];CloudData d=points(1000);
    SwingUtilities.invokeAndWait(()->{var v=new CloudView();holder[0]=v;v.setSize(320,240);v.setData(d,true);for(int i=0;i<60;i++)v.setView(v.view.orbit(10,10),true);v.setView(v.view.withRoll(25),false);});
    CloudView v=holder[0];long deadline=System.nanoTime()+TimeUnit.SECONDS.toNanos(20);
    while(v.frameShown==null && System.nanoTime()<deadline)Thread.sleep(20);
    SwingUtilities.invokeAndWait(()->{
      check(v.frameShown!=null,"latest asynchronous frame published");near(v.frameShown.request().view().roll(),25,"stale render cannot replace newest view");
      var before=v.view;v.remember();v.setView(v.view.withRoll(80),false);v.undo();near(v.view.roll(),before.roll(),"view history undo");v.redo();near(v.view.roll(),80,"view history redo");
      var request=v.frameShown.request();long[] scaled=new long[640*480];Arrays.fill(scaled,CloudRenderer.EMPTY);
      scaled[240*640+326]=(long)Float.floatToIntBits(100)<<32|7;
      v.frameShown=new CloudRenderer.Frame(640,480,new int[scaled.length],scaled,request);
      check(v.pickId(160,120)==7,"pick radius scales with physical render resolution");
      var picked=new java.util.concurrent.atomic.AtomicReference<CloudView.Pick>();
      var cameraPicked=new java.util.concurrent.atomic.AtomicBoolean();
      d.origin[0]=600000;d.origin[1]=6900000;d.origin[2]=100;
      d.cameras.add(new CloudData.Cam("overlapping.jpg",request.view().target(),new double[]{1,0,0,0,1,0,0,0,1},100,200,200,null,null));
      v.selected=picked::set;v.cameraSelected=c->cameraPicked.set(true);v.setTool(CloudView.Tool.INSPECT);
      check(v.getCursor().getType()==java.awt.Cursor.CROSSHAIR_CURSOR,"inspect tool uses a crosshair");
      v.dispatchEvent(new MouseEvent(v,MouseEvent.MOUSE_CLICKED,System.currentTimeMillis(),0,160,120,1,false,MouseEvent.BUTTON1));
      check(picked.get()!=null && picked.get().id()==7 && !cameraPicked.get(),"inspect selects a point even under a camera frustum");
      near(picked.get().world()[0],600007,"inspect publishes world X");
      near(picked.get().world()[1],6900000,"inspect publishes world Y");
      near(picked.get().world()[2],100,"inspect publishes world Z");
      var stable=v.view;
      v.dispatchEvent(new MouseEvent(v,MouseEvent.MOUSE_PRESSED,System.currentTimeMillis(),0,160,120,1,false,MouseEvent.BUTTON1));
      v.dispatchEvent(new MouseEvent(v,MouseEvent.MOUSE_DRAGGED,System.currentTimeMillis(),MouseEvent.BUTTON1_DOWN_MASK,190,140,0,false,MouseEvent.NOBUTTON));
      check(v.view==stable,"inspect left drag does not accidentally orbit the scene");
      v.dispatchEvent(new MouseEvent(v,MouseEvent.MOUSE_RELEASED,System.currentTimeMillis(),0,190,140,1,false,MouseEvent.BUTTON1));
      v.setData(points(0),false);check(v.frameShown==null,"data replacement clears obsolete picking buffer");v.dispose();
    });
    check(v.renderer.awaitTermination(10,TimeUnit.SECONDS),"renderer exits on close");
  }
  static void preview(Path root) throws Exception {
    Path run=Files.createDirectories(root.resolve("runs/terrain")),out=Files.createDirectories(run.resolve("07_results"));
    try(CloudWriter cloud=new CloudWriter(out,1,null,true)) {
      cloud.writePly(false);
      for(int y=0;y<600;y++)for(int x=0;x<700;x++) {
        double z=8*Math.sin(x*.018)*Math.cos(y*.014)+(x>250&&x<350&&y>250&&y<350?20:0);
        cloud.point(600000+x*.2,6900000+y*.2,100+z,80+x%150,100+y%120,80,new double[]{0,0,1},new SurfaceAttributes(.1,.01,.8,1,20,0,false));
      }
    }
    Path second=Files.createDirectories(root.resolve("runs/comparison/07_results"));Files.copy(out.resolve("dense.las"),second.resolve("dense.las"));
    SwingUtilities.invokeLater(()->new CloudWindow(new ProjectFiles.Run("terrain",run,null,"complete",List.of(),420000L,List.of(),List.of(),List.of()),null));
  }
}
