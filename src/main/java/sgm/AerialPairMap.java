package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.*;
import java.nio.file.*;

/** 20 bytes/pixel: disparity, left/right confidence, photometric score and RGB. No XYZ spool. */
final class AerialPairMap implements AutoCloseable, AerialTileStereo.Sink {
  static final int BYTES=20;
  final DenseReconstruction.Rectified geometry;
  final int source,width,height;
  final Path path;
  private final Options o;
  private final DepthFusion fusion;
  private final FileChannel file;
  private final double[] inverseRectification,inverseCamera;
  private final double[] centerA,centerB;
  private final double minimumAngleCosine;
  long valid,sampled,seamPixels;

  AerialPairMap(Options options,DenseReconstruction.Rectified geometry,int source,
      DepthFusion fusion,Path directory) throws IOException {
    o=options;this.geometry=geometry;this.source=source;this.fusion=fusion;
    width=geometry.vertical ? geometry.a.height : geometry.a.width;
    height=geometry.vertical ? geometry.a.width : geometry.a.height;
    Files.createDirectories(directory);
    path=Files.createTempFile(directory,"aerial-depth-",".tiles");
    try {file=FileChannel.open(path,StandardOpenOption.READ,StandardOpenOption.WRITE);}
    catch(IOException e){Files.deleteIfExists(path);throw e;}
    inverseRectification=Math3.transpose(geometry.r1);
    inverseCamera=Math3.transpose(geometry.a.camera.rotation);
    centerA=geometry.a.camera.center();centerB=geometry.b.camera.center();
    minimumAngleCosine=Math.cos(Math.toRadians(o.minAngle));
  }

  record Pixel(float disparity,float confidence,float rightConfidence,float photo,int rgb) {}
  record Block(ByteBuffer data) {
    Pixel get(int index) {
      int i=index*BYTES;float d=data.getFloat(i);
      return new Pixel(d==0?Float.NaN:d,data.getFloat(i+4),data.getFloat(i+8),data.getFloat(i+12),data.getInt(i+16));
    }
  }
  public double seamError(AerialTileStereo.Match match) throws IOException {
    var c=match.prepared().plan().core();
    int checked=0,bad=0,overlap=Math.min(8,o.aerialOverlap);
    // Compare the new tile's context with the already committed neighbour's core.
    for(int y=Math.max(0,c.y()-overlap);y<c.y()+c.height();y++) {
      int x0=Math.max(0,c.x()-overlap),x1=y<c.y()?c.x()+c.width():c.x();
      if(x1==x0)continue;
      var previous=block(x0,y,x1-x0,1);
      for(int x=x0;x<x1;x++) {
        float a=previous.get(x-x0).disparity(),b=match.disparity(x,y);
        if(Float.isFinite(a) && Float.isFinite(b)) {checked++;if(Math.abs(a-b)>1.0)bad++;}
      }
    }
    return checked<16 ? 0 : bad/(double)checked;
  }

  public void accept(AerialTileStereo.Match match,boolean rejectSeam) throws IOException {
    var p=match.prepared().plan();var c=p.core();var result=match.result();
    byte[] colors=match.prepared().left().color();
    ByteBuffer row=ByteBuffer.allocate(c.width()*BYTES).order(ByteOrder.LITTLE_ENDIAN);
    for(int y=c.y();y<c.y()+c.height();y++) {
      row.clear();
      for(int x=c.x();x<c.x()+c.width();x++) {
        int i=(y-p.y())*p.width()+x-p.x();
        float d=match.disparity(x,y);
        if(rejectSeam && ((c.x()>0 && x<c.x()+o.aerialOverlap)
            || (c.y()>0 && y<c.y()+o.aerialOverlap))) {d=Float.NaN;seamPixels++;}
        double[] world=world(x,y,d);
        if(world==null)d=Float.NaN;
        else {
          fusion.add(source,world);valid++;
          if(x%o.step==0 && y%o.step==0)sampled++;
        }
        int rgb=(colors[3*i+2]&255)<<16 | (colors[3*i+1]&255)<<8 | colors[3*i]&255;
        row.putFloat(d).putFloat(value(result.confidence,i)).putFloat(value(result.rightConfidence,i))
            .putFloat(value(result.correlation,i)).putInt(rgb);
      }
      row.flip();long offset=((long)y*width+c.x())*BYTES;
      while(row.hasRemaining())file.write(row,offset+row.position());
    }
  }

  static float value(float[] data,int i) {return data==null ? Float.NaN : data[i];}

  double[] world(int x,int y,float disparity) {
    if(!Float.isFinite(disparity) || Math.abs(disparity)<1e-6)return null;
    double xx=geometry.vertical ? y : x, yy=geometry.vertical ? x : y;
    double[] q=geometry.q;
    double den=q[12]*xx+q[13]*yy+q[14]*disparity+q[15];
    if(Math.abs(den)<1e-12)return null;
    double rx=(q[0]*xx+q[1]*yy+q[2]*disparity+q[3])/den,
        ry=(q[4]*xx+q[5]*yy+q[6]*disparity+q[7])/den,
        rz=(q[8]*xx+q[9]*yy+q[10]*disparity+q[11])/den;
    if(!Double.isFinite(rx+ry+rz) || rz<=0 || rz>o.maxDepth)return null;
    double[] r=inverseRectification,t=geometry.a.camera.translation,v=inverseCamera;
    double ax=r[0]*rx+r[1]*ry+r[2]*rz-t[0],ay=r[3]*rx+r[4]*ry+r[5]*rz-t[1],
        az=r[6]*rx+r[7]*ry+r[8]*rz-t[2];
    double[] world={v[0]*ax+v[1]*ay+v[2]*az,v[3]*ax+v[4]*ay+v[5]*az,v[6]*ax+v[7]*ay+v[8]*az};
    Camera b=geometry.b.camera;
    if(b.rotation[6]*world[0]+b.rotation[7]*world[1]+b.rotation[8]*world[2]+b.translation[2]<=0)return null;
    if(o.minAngle>0) {
      double a0=world[0]-centerA[0],a1=world[1]-centerA[1],a2=world[2]-centerA[2],
          b0=world[0]-centerB[0],b1=world[1]-centerB[1],b2=world[2]-centerB[2];
      double cosine=(a0*b0+a1*b1+a2*b2)/Math.sqrt((a0*a0+a1*a1+a2*a2)*(b0*b0+b1*b1+b2*b2));
      if(cosine>minimumAngleCosine)return null;
    }
    return world;
  }

  /** Read one bounded spatial block; callers never allocate a complete disparity map. */
  Block block(int x,int y,int w,int h) throws IOException {
    ByteBuffer data=ByteBuffer.allocate(w*h*BYTES).order(ByteOrder.LITTLE_ENDIAN);
    for(int yy=0;yy<h;yy++) {
      ByteBuffer row=data.slice(yy*w*BYTES,w*BYTES);
      long offset=((long)(y+yy)*width+x)*BYTES;
      while(row.hasRemaining()) {
        int n=file.read(row,offset+row.position());
        if(n<=0)throw new IOException("Incomplete aerial compact map");
      }
    }
    return new Block(data);
  }
  public void close() throws IOException {
    try{file.close();}finally{Files.deleteIfExists(path);}
  }
}
