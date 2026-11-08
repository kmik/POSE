package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Specialized aerial pipeline. Stereo is GPU tiled; indexed depth evidence is fused in blocks. */
final class AerialTiledReconstruction implements AutoCloseable {
  interface Geometry {
    DenseReconstruction.Rectified rectify(ImageFrame a,ImageFrame b,int source) throws IOException;
  }
  private final Options o;
  private final List<ImageFrame> frames;
  private final Project project;
  private final Crs crs;
  private final DepthFusion fusion;
  private final Geometry geometry;
  private final List<AerialPairMap> maps=new ArrayList<>();
  private final List<Object> reports=new ArrayList<>();
  private long candidates,accepted,rejected,compactBytes,valid,seamPixels;
  private final Map<String,Long> decisions=new TreeMap<>();

  AerialTiledReconstruction(Options options,List<ImageFrame> frames,Project project,Crs crs,
      DepthFusion fusion,Geometry geometry) {
    o=options;this.frames=frames;this.project=project;this.crs=crs;this.fusion=fusion;this.geometry=geometry;
  }

  long run(List<FeatureTracks.Pair> pairs) throws IOException {
    // Fail before decoding large rasters if no eligible GPU is available. No CPU substitution.
    GpuSgm.selectedDevice(o);
    RasterCache.clear();
    for(var frame:frames)frame.releaseRasters();
    try(var progress=WorkProgress.start("Aerial dense matching pairs",pairs.size(),"pairs")) {
      int processed=0;
      for(var pair:pairs) {
        ImageFrame a=frames.get(pair.a),b=frames.get(pair.b);
        if(!a.camera.registered || !b.camera.registered || a.camera.denseExcluded || b.camera.denseExcluded)continue;
        int source=fusion.addPair(a.id,b.id);
        progress.detail("pair "+a.id+"-"+b.id);
        try(var r=geometry.rectify(a,b,source)) {
          Path directory=o.output.resolve(String.format(Locale.ROOT,"05_stereo/pair_%04d_%04d",r.a.id,r.b.id));
          AerialPairMap map=new AerialPairMap(o,r,source,fusion,directory);maps.add(map);
          var stereo=new AerialTileStereo(o,new AerialRectifier(r,o),map.width,map.height);
          stereo.run(map);
          candidates+=map.sampled;valid+=map.valid;seamPixels+=map.seamPixels;
          compactBytes+=AerialPairMap.BYTES*(long)map.width*map.height;
          Object report=Json.object("image_a",r.a.id,"image_b",r.b.id,"valid",map.valid,
              "sampled_candidates",map.sampled,"compact_map_bytes",AerialPairMap.BYTES*(long)map.width*map.height,
              "seam_rejected_pixels",map.seamPixels,"stereo",stereo.report());
          Json.write(directory.resolve("aerial_tiled.json"),report);reports.add(report);
        }
        progress.update(++processed);
      }
      progress.done();
    }
    if(candidates==0)throw new IOException("Aerial tiled stereo produced no valid candidates; inspect coarse ranges, calibration and overlap");
    if(project!=null) {
      project.stage("05_stereo","complete","GPU-resident aerial tiles; compact disparity/confidence maps");
      project.stage("06_fusion","running","Bounded spatial-block depth corroboration");
    }
    // The same geometric/visibility/uncertainty checks as reference fusion; no all-candidate
    // XYZ file, image-fusion camera arrays or global consolidation/deduplication hash.
    fusion.prepare(candidates);
    int side=Math.min(128,o.aerialTileSize);
    try(var output=new FusionOutput(o,fusion,project,crs);
        var progress=WorkProgress.start("Aerial block fusion",candidates,"candidates")) {
      long seen=0;
      for(var map:maps)for(int y=0;y<map.height;y+=side)for(int x=0;x<map.width;x+=side) {
        int w=Math.min(side,map.width-x),h=Math.min(side,map.height-y);
        var block=map.block(x,y,w,h);
        double[][] points=new double[w*h][];
        int[] sources=new int[w*h],pixels=new int[w*h],indices=new int[w*h];int n=0;
        for(int yy=0;yy<h;yy++)for(int xx=0;xx<w;xx++) {
          int gx=x+xx,gy=y+yy;
          if(gx%o.step!=0 || gy%o.step!=0)continue;
          int i=yy*w+xx;double[] world=map.world(gx,gy,block.get(i).disparity());
          if(world==null)continue;
          points[n]=world;sources[n]=map.source;pixels[n]=gy*map.width+gx;indices[n]=i;n++;
        }
        if(n==0)continue;
        var results=fusion.fuseBatch(sources,points,n);
        for(int i=0;i<n;i++) {
          var p=block.get(indices[i]);
          var result=TwoViewConfidence.apply(results[i],p.confidence(),p.rightConfidence(),o.twoViewConfidenceThreshold);
          result.matchingConfidence=p.confidence();result.photometric=p.photo();
          result.rightMatchingConfidence=p.rightConfidence();
          result.confidence=PointConfidence.finalScore(result.matchingConfidence,result.relativeSigma);
          results[i]=result;
          decisions.merge(result.status,1L,Long::sum);
        }
        DepthFusion.SurfaceSample[] surfaces=o.pointAttributes.equals("surface")
            ? fusion.surfaceBatch(sources,results,n) : null;
        for(int i=0;i<n;i++,seen++) {
          var result=results[i];
          if(!result.accepted()){rejected++;continue;}
          output.add(seen,map.source,pixels[i],block.get(indices[i]).rgb(),result,surfaces==null ? null : surfaces[i]);
        }
        progress.update(seen);
      }
      if(seen!=candidates)throw new IOException("Aerial compact map candidate count changed during replay");
      if(output.count()==0)throw new IOException("No aerial points passed block fusion: "+decisions);
      output.finish();accepted=output.count();progress.done();
    }
    Files.createDirectories(o.output.resolve("06_fusion"));
    if(DepthOutlierFilter.enabled(o))
      Json.write(o.output.resolve("06_fusion/backend.json"),fusion.backendReport());
    Json.write(o.output.resolve("06_fusion/aerial_tiled.json"),Json.object(
        "pipeline","aerial-tiled","fusion","spatial-block-reference","matching_backend","gpu",
        "candidate_spool_bytes",0,"aggregate_spill_bytes",0,"compact_map_bytes",compactBytes,
        "valid_stereo_pixels",valid,"sampled_candidates",candidates,"accepted",accepted,
        "fusion_rejected",rejected,"decisions",decisions,"seam_rejected_pixels",seamPixels,"pairs",reports));
    if(project!=null) {
      fusion.export(project.data);project.data.finishFusion();
      project.stage("06_fusion","complete",accepted+" points retained by bounded block fusion");
      project.stage("07_results","running","Publishing aerial cloud and report");
    }
    return accepted;
  }

  String summary() {
    return "Dense pipeline=aerial-tiled\nMatching backend=OpenCL GPU\nApproximate tile-local SGM=true"
        +"\nRaw stereo observations="+valid+"\nDense candidates="+candidates
        +"\nFusion rejected="+rejected+"\nExported points="+accepted
        +"\nCompact map bytes="+compactBytes+"\nAggregate spill bytes=0\nCandidate XYZ spool bytes=0"
        +"\nSeam rejected pixels="+seamPixels;
  }
  public void close() throws IOException {
    IOException failure=null;
    for(var map:maps)try{map.close();}catch(IOException e){if(failure==null)failure=e;else failure.addSuppressed(e);}
    try {
      fusion.close();
    }catch(IOException e){if(failure==null)failure=e;else failure.addSuppressed(e);}
    try {if(!(o.diagnostics && o.storage.equals("debug")))fusion.discardStorage();}
    catch(IOException e){if(failure==null)failure=e;else failure.addSuppressed(e);}
    if(failure!=null)throw failure;
  }
}
