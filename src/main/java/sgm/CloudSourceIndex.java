package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Small in-memory index of LAS record-block bounds; never copies or modifies the source cloud. */
final class CloudSourceIndex {
  record Stamp(Path path,long size,long modified) {}
  record Block(Path file,long start,int count,long firstId,CloudDisplay.Box bounds) {}
  final List<Stamp> stamps;
  final List<Block> blocks=new ArrayList<>();
  Node root;
  static final class Node {
    CloudDisplay.Box bounds;
    Node left,right;
    Block block;
  }
  CloudSourceIndex(List<Path> files) throws IOException {
    List<Stamp> s=new ArrayList<>();
    for(Path p:files) s.add(new Stamp(p,Files.size(p),Files.getLastModifiedTime(p).toMillis()));
    stamps=List.copyOf(s);
  }
  boolean valid(List<Path> files) throws IOException {
    return stamps.equals(new CloudSourceIndex(files).stamps);
  }
  void finish() { root=build(new ArrayList<>(blocks),0,blocks.size()); }
  private Node build(List<Block> b,int lo,int hi) {
    if(lo==hi) return null;
    Node n=new Node();
    double[] min={Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY};
    double[] max={Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
    for(int i=lo;i<hi;i++) {
      var v=b.get(i).bounds();
      min[0]=Math.min(min[0],v.x0());min[1]=Math.min(min[1],v.y0());min[2]=Math.min(min[2],v.z0());
      max[0]=Math.max(max[0],v.x1());max[1]=Math.max(max[1],v.y1());max[2]=Math.max(max[2],v.z1());
    }
    n.bounds=new CloudDisplay.Box(min[0],min[1],min[2],max[0],max[1],max[2]);
    if(hi-lo==1) {n.block=b.get(lo);return n;}
    int a=max[1]-min[1]>max[0]-min[0]?1:0;if(max[2]-min[2]>max[a]-min[a])a=2;
    final int axis=a;
    b.subList(lo,hi).sort(Comparator.comparingDouble(v->v.bounds().centre()[axis]));
    int mid=(lo+hi)/2;n.left=build(b,lo,mid);n.right=build(b,mid,hi);return n;
  }
  List<Block> query(CloudDisplay.Box box) {
    List<Block> out=new ArrayList<>();query(root,box,out);
    out.sort(Comparator.comparing((Block b)->b.file().toString()).thenComparingLong(Block::start));
    return out;
  }
  private void query(Node n,CloudDisplay.Box box,List<Block> out) {
    if(n==null || !n.bounds.intersects(box)) return;
    if(n.block!=null)out.add(n.block);else {query(n.left,box,out);query(n.right,box,out);}
  }
}
