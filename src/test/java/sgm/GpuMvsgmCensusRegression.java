package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.io.*;
import java.lang.foreign.*;
import java.nio.file.Path;
import java.util.*;
import java.util.concurrent.TimeUnit;

/** Every-pixel census, CPU warp preservation, halos/offsets, lazy fallback and reservation growth. */
final class GpuMvsgmCensusRegression {
  private static int checks;
  private static final long SENTINEL = 0x13579bdf2468ace0L;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    if (args.length > 0 && args[0].equals("build")) buildFailure();
    else if (args.length > 0) failure();
    else System.out.println("PASS GPU mvsgm census: " + run());
  }

  private static byte[] gray(int w, int h) {
    byte[] g = new byte[w*h];
    Random r = new Random(8021);
    for (int y=0;y<h;y++) for (int x=0;x<w;x++)
      g[y*w+x] = (byte) (x<w/4 ? 127 : x<w/2 ? (x+y)%2*255 : r.nextInt(256));
    return g;
  }

  private static Camera camera(int w, int h, double angle, double scale) {
    Camera c = new Camera(150*scale, 143*scale, (w-1)*.5, (h-1)*.5);
    double co=Math.cos(angle), si=Math.sin(angle);
    c.rotation = new double[] {co,-si,0,si,co,0,0,0,1};
    c.translation = new double[] {-1.5, .2, .1};
    return c;
  }

  private static void canvas(GpuSgm.Compute cl, MemorySegment kernel, Options o, byte[] g,
      int w, int h, int window, boolean reference, boolean small) {
    long[] expected=CensusSgm.census(g,w,h,window)[0];
    int prefix=13, suffix=17;
    long outputBytes=8L*(prefix+g.length+suffix);
    long scratch=reference?g.length:Math.min(g.length,GpuSgm.budget(cl.device,o)-outputBytes);
    if (!small) scratch=reference?g.length:Math.min(g.length,GpuMvsgmCensus.MAX_SCRATCH);
    check(outputBytes+scratch<=GpuSgm.budget(cl.device,o),"test buffers respect the GPU budget");
    try(var reservation=GpuMemoryPool.reserveAuxiliary(outputBytes+scratch); Arena a=Arena.ofConfined()) {
      check(reservation!=null,"test memory reserved");
      MemorySegment source=cl.buffer(scratch), out=cl.buffer(outputBytes);
      try {
        long[] init=new long[prefix+g.length+suffix]; Arrays.fill(init,SENTINEL);
        cl.transfer(out,0,a.allocateFrom(JAVA_LONG,init),false);
        if(reference) cl.transfer(source,0,a.allocateFrom(JAVA_BYTE,g),false);
        long before=GpuMvsgmCensus.bands;
        GpuMvsgmCensus.generate(cl,kernel,source,out,prefix,g,w,h,window,reference,scratch,()->expected);
        if(small) check(GpuMvsgmCensus.bands-before>1,"small budget splits a canvas into halo bands");
        MemorySegment result=a.allocate(outputBytes,8); cl.transfer(out,0,result,true);
        int mismatch=0;
        for(int i=0;i<init.length;i++) {
          long want=i<prefix || i>=prefix+g.length?SENTINEL:expected[i-prefix];
          if(result.getAtIndex(JAVA_LONG,i)!=want) mismatch++;
        }
        check(mismatch==0,"all descriptors, border zeros and offset sentinels equal: "+w+"x"+h+" window "+window);
      } finally {cl.releaseBuffer(source);cl.releaseBuffer(out);}
    }
  }

  static int run() throws Exception {
    checks=0;
    Options o=new Options(); o.compute="auto";
    Options cpu=o.copy();cpu.compute="cpu";
    check(!GpuMvsgmCensus.enabled(cpu),"--compute cpu leaves census on the CPU");
    for(String flag:List.of("sgm.cpuMvsgm","sgm.cpuMvsgmCensus")) {
      String previous=System.getProperty(flag);
      try {System.setProperty(flag,"true");check(!GpuMvsgmCensus.enabled(o),flag+" leaves census on the CPU");}
      finally {if(previous==null)System.clearProperty(flag);else System.setProperty(flag,previous);}
    }
    Options tiny=o.copy();tiny.gpuMemoryMb=1;
    check(!GpuMvsgmCensus.enabled(tiny),"1 MiB selects CPU census");
    check(GpuMvsgmCensus.bandRows(601,401,7,601L*(64+6))==64,"halo rows consume scratch bytes");
    check(GpuMvsgmCensus.bandRows(601,401,7,601L*(63+6))==0,"too little scratch selects CPU");
    check(GpuMvsgmCensus.bandRows(17,5,7,85)==5,"a complete small canvas needs no extra halo");
    if(GpuSgm.devices().isEmpty()) {
      System.out.println("GpuMvsgmCensusRegression skipped GPU checks: no OpenCL device");return checks;
    }
    // The requested regression device must support FP64, although census itself uses integers.
    String fp64="""
        #pragma OPENCL EXTENSION cl_khr_fp64 : enable
        __kernel void probe(__global double *a) { a[0]=sqrt(a[0]+1.0); }
        """;
    try(var probe=new GpuSgm.Compute(o,fp64)) {probe.kernel("probe");}
    catch(RuntimeException unavailable) {
      System.out.println("GpuMvsgmCensusRegression skipped GPU checks: no OpenCL FP64 device");return checks;
    }
    try(var cl=new GpuSgm.Compute(o,GpuMvsgmCensus.SOURCE)) {
      MemorySegment kernel=cl.kernel("mvcensus");
      int w=131,h=99;byte[] g=gray(w,h),mask=new byte[w*h];Arrays.fill(mask,(byte)1);
      for(int y=20;y<37;y++)for(int x=17;x<48;x++)mask[y*w+x]=0;
      for(int window:new int[]{3,5,7}) {
        var eager=new MultiViewSgm.Reference(w,h,g,mask,window);
        var lazy=new MultiViewSgm.Reference(w,h,g,mask,window,true);
        check(lazy.census==null && Arrays.equals(eager.use,lazy.use),"deferred reference keeps the same CPU masks");
        canvas(cl,kernel,o,g,w,h,window,true,false);
        check(Arrays.equals(eager.census,lazy.cpuCensus()),"lazy reference fallback equals the CPU");
        for(double angle:new double[]{0,.17,-.4,Math.PI}) for(double scale:new double[]{.75,1,1.2}) {
          Camera ref=camera(w,h,0,1),other=camera(w,h,angle,scale);
          int[] rectangle={-13,-7,w+27,h+15};
          var a=MultiViewSgm.warp(ref,other,1,rectangle,g,mask,0,0,w,h,window);
          var b=MultiViewSgm.warp(ref,other,1,rectangle,g,mask,0,0,w,h,window,true);
          check(b.census==null && Arrays.equals(a.gray,b.gray) && Arrays.equals(a.raw,b.raw)
              && Arrays.equals(a.valid,b.valid),"deferred census preserves CPU warp and all invalid regions");
          for(int level=0;level<3;level++) {
            canvas(cl,kernel,o,b.gray,b.w,b.h,window,false,false);
            check(Arrays.equals(a.census,b.cpuCensus()),"CPU fallback equals the eager descriptor at level "+level);
            a=a.coarser(window);b=b.coarser(window);
          }
        }
      }
      Options small=o.copy();small.gpuMemoryMb=2;
      for(int window:new int[]{3,5,7}) canvas(cl,kernel,small,gray(601,401),601,401,window,false,true);
      // The resident reference spans multiple launches, including a partial row at the split.
      canvas(cl,kernel,o,gray(2081,2051),2081,2051,5,true,false);
    }
    growth(o);
    switches(o);
    for(String mode:List.of("failure","build")) {
      Process child=new ProcessBuilder(Path.of(System.getProperty("java.home"),"bin","java.exe").toString(),
          "--enable-native-access=ALL-UNNAMED","-cp",System.getProperty("java.class.path"),
          GpuMvsgmCensusRegression.class.getName(),mode).redirectErrorStream(true).start();
      if(!child.waitFor(45,TimeUnit.SECONDS)){child.destroyForcibly();throw new AssertionError("census failure child timed out");}
      String output=new String(child.getInputStream().readAllBytes(),java.nio.charset.StandardCharsets.UTF_8);
      check(child.exitValue()==0 && output.contains(mode+" fallback passed"),mode+" disables GPU census permanently: "+output);
    }
    return checks;
  }

  private static void growth(Options o) {
    int w=73,h=67;byte[] g=gray(w,h),mask=new byte[w*h];Arrays.fill(mask,(byte)1);
    var ref=new MultiViewSgm.Reference(w,h,g,mask,5,true);
    var view=MultiViewSgm.warp(camera(w,h,0,1),camera(w,h,.1,1),1,
        new int[]{0,0,w,h},g,mask,0,0,w,h,5,true);
    var labels=new MultiViewSgm.Labels(.01,.001,32,.01,.041,100);
    var plan=new MultiViewSgm.Plan(0,List.of(new MultiViewSgm.Neighbor(1,1,100,7)),labels,ref,new MultiViewSgm.Warped[]{view});
    var mv=new MultiViewSgm(o);
    var narrow=new DisparityBands(w,h,32);
    for(int p=0;p<w*h;p++)if(ref.use[p])narrow.count[p]=8;
    narrow.changed();
    var full=DisparityBands.full(w,h,32,ref.use);
    long initial=GpuMultiViewSgm.deviceBytes(mv,plan,narrow.total());
    long larger=GpuMultiViewSgm.deviceBytes(mv,plan,full.total());
    long poolBytes=larger+4096;
    try(var pool=GpuMemoryPool.install(poolBytes); var session=new GpuMultiViewSgm.Session(o,mv,plan,initial)) {
      int[] expected=mv.cpuSummary(plan,narrow);
      check(Arrays.equals(expected,session.summary(narrow)),"deferred-plan GPU summary equals CPU");
      try(var blocker=GpuMemoryPool.reserveAuxiliary(poolBytes-initial-1024)) {
        check(blocker!=null && !session.reserve(larger),"growth refuses pool contention without waiting while holding memory");
      }
      check(session.reserve(larger),"same input session grows after memory becomes available");
      long before=GpuMvsgmCensus.canvases;
      check(Arrays.equals(session.summary(full),mv.cpuSummary(plan,full)),"widened summary equals CPU");
      check(GpuMvsgmCensus.canvases==before,"retry reuses device descriptors");
    }
  }

  private static void failure() {
    Options o=new Options();o.compute="auto";
    String bad=GpuMvsgmCensus.SOURCE.replace("out[(size_t)offset+(size_t)y*w+x] = word;",
        "out[(size_t)offset+(size_t)y*w+x] = 0;");
    boolean caught=false;
    try(var cl=new GpuSgm.Compute(o,bad);Arena a=Arena.ofConfined()) {
      byte[] gray=gray(83,79);MemorySegment source=cl.buffer(gray.length),out=cl.buffer(8L*gray.length);
      cl.transfer(source,0,a.allocateFrom(JAVA_BYTE,gray),false);
      try {
        GpuMvsgmCensus.generate(cl,cl.kernel("mvcensus"),source,out,0,gray,83,79,5,true,gray.length,
            ()->CensusSgm.census(gray,83,79,5)[0]);
      }catch(IllegalStateException mismatch) {
        ByteArrayOutputStream bytes=new ByteArrayOutputStream();PrintStream original=System.out;
        try(PrintStream capture=new PrintStream(bytes)) {
          System.setOut(capture);GpuMvsgmCensus.failed(mismatch);GpuMvsgmCensus.failed(mismatch);
        }finally{System.setOut(original);}
        check(bytes.toString().split("WARNING:",-1).length==2,"one warning per process");
        caught=true;
      }
    }
    check(caught && !GpuMvsgmCensus.enabled(o),"mismatch permanently disables census");
    System.out.println("failure fallback passed");
  }

  private static void buildFailure() {
    Options o=new Options();o.compute="auto";
    // A duplicate name deliberately fails the combined program; the base program remains valid.
    String base="__kernel void mvcensus(__global int *out) { if(get_global_id(0)==0) out[0]=17; }"
        + System.lineSeparator();
    ByteArrayOutputStream bytes=new ByteArrayOutputStream();PrintStream original=System.out;
    try(PrintStream capture=new PrintStream(bytes)) {
      System.setOut(capture);
      for(int attempt=0;attempt<2;attempt++) {
        try(var cl=GpuMvsgmCensus.compute(o,base);Arena a=Arena.ofConfined()) {
          MemorySegment out=cl.buffer(4),host=a.allocate(4,4);
          cl.run(cl.kernel("mvcensus"),1,out);cl.transfer(out,0,host,true);
          check(host.get(JAVA_INT,0)==17,"base GPU program remains usable after census build failure");
        }
      }
    } finally {System.setOut(original);}
    check(!GpuMvsgmCensus.enabled(o),"build failure disables census for the process");
    check(bytes.toString().split("WARNING:",-1).length==2,"build failure warns once and is not retried");
    System.out.println("build fallback passed");
  }

  /** Exercise each switch through the solve, including CPU descriptor materialisation. */
  private static void switches(Options options) {
    int w=73,h=67;byte[] g=gray(w,h),mask=new byte[w*h];Arrays.fill(mask,(byte)1);
    var labels=new MultiViewSgm.Labels(.01,.001,32,.01,.041,100);
    int[] expected=null;
    for(String mode:List.of("cpu","sgm.cpuMvsgm","sgm.cpuMvsgmCensus","tiny")) {
      Options o=options.copy();
      if(mode.equals("cpu"))o.compute="cpu";
      if(mode.equals("tiny"))o.gpuMemoryMb=1;
      String flag=mode.startsWith("sgm.")?mode:null;
      String previous=flag==null?null:System.getProperty(flag);
      try {
        if(flag!=null)System.setProperty(flag,"true");
        var ref=new MultiViewSgm.Reference(w,h,g,mask,5,true);
        var view=MultiViewSgm.warp(camera(w,h,0,1),camera(w,h,.1,1),1,
            new int[]{0,0,w,h},g,mask,0,0,w,h,5,true);
        var plan=new MultiViewSgm.Plan(0,List.of(new MultiViewSgm.Neighbor(1,1,100,7)),labels,ref,new MultiViewSgm.Warped[]{view});
        var bands=DisparityBands.full(w,h,32,ref.use);
        var mv=new MultiViewSgm(o);
        long before=GpuMvsgmCensus.canvases;
        int[] actual=mv.solveLevel(plan,bands,new long[1]);
        if(expected==null)expected=actual;
        check(Arrays.equals(expected,actual),mode+" preserves the CPU summary");
        check(GpuMvsgmCensus.canvases==before,mode+" executes no device census");
        check(ref.census!=null && view.census!=null,mode+" materialises deferred CPU descriptors");
        if(mode.equals("sgm.cpuMvsgmCensus") || mode.equals("tiny"))
          check(mv.lastBackend.equals("gpu"),mode+" disables census alone while this small SGM fits the device");
        else check(mv.lastBackend.equals("cpu"),mode+" selects the CPU SGM path");
      } finally {
        if(flag!=null) {if(previous==null)System.clearProperty(flag);else System.setProperty(flag,previous);}
      }
    }
  }
}
