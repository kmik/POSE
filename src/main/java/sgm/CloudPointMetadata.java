package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.*;
import java.nio.file.*;
import java.util.*;

/** Lightweight, cancellable provenance access; sparse seek bookmarks avoid repeated full scans. */
final class CloudPointMetadata {
  record Evidence(double confidence,int views,int pairs,double sigma,BitSet cameras) {}
  final Path file;
  long size=-1,modified=-1;
  final NavigableMap<Long,Long> bookmarks=new TreeMap<>();
  CloudPointMetadata(Path file){this.file=file;}

  synchronized Evidence find(long wanted) throws IOException {
    long bytes=Files.size(file),time=Files.getLastModifiedTime(file).toMillis();
    if(bytes!=size || time!=modified){bookmarks.clear();size=bytes;modified=time;}
    var floor=bookmarks.floorEntry(wanted);
    try(Scan scan=new Scan(file,floor==null?8:floor.getValue())) {
      long records=0;
      while(scan.next()) {
        if((records++&4095)==0)bookmarks.put(scan.id,scan.start);
        if(scan.id>wanted)return null;
        if(scan.id==wanted)return scan.evidence(true);
        scan.skipCameras();
      }
    }
    return null;
  }

  static float[][] samples(Path file,long[] ids,int count) throws IOException {
    float[] views=new float[count],confidence=new float[count];Arrays.fill(views,Float.NaN);Arrays.fill(confidence,Float.NaN);
    if(count==0)return new float[][]{views,confidence};
    try(Scan scan=new Scan(file,8)) {
      int sample=0;
      while(sample<count && scan.next()) {
        while(sample<count && ids[sample]<scan.id)sample++;
        if(sample<count && ids[sample]==scan.id){var e=scan.evidence(false);views[sample]=e.views();confidence[sample]=(float)e.confidence();sample++;}
        else scan.skipCameras();
      }
    }
    return new float[][]{views,confidence};
  }

  private static final class Scan implements AutoCloseable {
    final DataInputStream in;
    final ByteBuffer buffer;
    final byte[] fixed;
    final int shift;
    long position,start,id;
    int cameraBytes;
    boolean pending;
    Scan(Path file,long offset) throws IOException {
      FileChannel channel=FileChannel.open(file);
      try {
        ByteBuffer magic=ByteBuffer.allocate(8);while(magic.hasRemaining())if(channel.read(magic)<0)throw new EOFException();
        long value=magic.flip().getLong();
        if(value!=PointRecords.MAGIC && value!=0x53474d5054530001L)throw new IOException("Unknown point metadata format");
        shift=value==PointRecords.MAGIC?0:-4;
        fixed=new byte[205+shift];buffer=ByteBuffer.wrap(fixed);
        channel.position(offset);in=new DataInputStream(new BufferedInputStream(Channels.newInputStream(channel),1<<20));position=offset;
      }catch(IOException|RuntimeException e){channel.close();throw e;}
    }
    boolean next() throws IOException {
      CloudData.cancelled();if(pending)skipCameras();
      start=position;int marker=in.read();position++;
      if(marker==0)return false;if(marker<0)throw new EOFException("Truncated point metadata");
      if(marker!=1)throw new IOException("Invalid point metadata marker");
      in.readFully(fixed);position+=fixed.length;id=buffer.getLong(0);cameraBytes=buffer.getInt(201+shift);
      if(cameraBytes<0 || cameraBytes>1<<20)throw new IOException("Invalid camera bitmap length");
      pending=true;return true;
    }
    void skipCameras() throws IOException {if(pending){in.skipNBytes(cameraBytes);position+=cameraBytes;pending=false;}}
    Evidence evidence(boolean cameras) throws IOException {
      BitSet ids=null;
      if(cameras){byte[] bitmap=new byte[cameraBytes];in.readFully(bitmap);position+=cameraBytes;pending=false;ids=BitSet.valueOf(bitmap);}
      else skipCameras();
      return new Evidence(buffer.getDouble(153+shift),buffer.getInt(28+shift),buffer.getInt(32+shift),buffer.getDouble(64+shift),ids);
    }
    public void close() throws IOException{in.close();}
  }
}
