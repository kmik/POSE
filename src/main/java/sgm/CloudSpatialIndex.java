package sgm;

import java.util.*;
import java.util.function.BooleanSupplier;

/** A bounded binary spatial hierarchy of loaded samples, with representative points per node. */
final class CloudSpatialIndex {
  final CloudData data;
  final int[] ids;
  final Node root;
  static final int LEAF = 1024, SAMPLE = 32;
  static final class Node {
    int start, end;
    CloudDisplay.Box bounds;
    Node left, right;
    int cost() { return left == null ? end-start : Math.min(SAMPLE,end-start); }
  }
  CloudSpatialIndex(CloudData d) {
    data=d; ids=new int[d.n];
    for(int i=0;i<ids.length;i++) ids[i]=i;
    root=build(0,d.n,0);
  }
  private Node build(int start,int end,int depth) {
    if(Thread.currentThread().isInterrupted()) throw new java.util.concurrent.CancellationException();
    Node n=new Node(); n.start=start;n.end=end;
    double[] lo={Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY};
    double[] hi={Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
    for(int k=start;k<end;k++) {
      int i=ids[k];
      for(int a=0;a<3;a++) {double v=axis(i,a);lo[a]=Math.min(lo[a],v);hi[a]=Math.max(hi[a],v);}
    }
    if(start==end) {lo=new double[3];hi=new double[3];}
    n.bounds=new CloudDisplay.Box(lo[0],lo[1],lo[2],hi[0],hi[1],hi[2]);
    if(end-start<=LEAF || depth>=32) return n;
    int a=hi[1]-lo[1]>hi[0]-lo[0]?1:0;
    if(hi[2]-lo[2]>hi[a]-lo[a]) a=2;
    double mid=(lo[a]+hi[a])/2;
    int l=start,r=end-1;
    while(l<=r) {if(axis(ids[l],a)<mid) l++; else {int v=ids[l];ids[l]=ids[r];ids[r--]=v;}}
    if(l==start || l==end) return n;
    n.left=build(start,l,depth+1); n.right=build(l,end,depth+1); return n;
  }
  private double axis(int i,int a) {return a==0?data.x[i]:a==1?data.y[i]:data.z[i];}

  int[] select(CloudRenderer.Projector p,CloudDisplay.Box clip,int budget,BooleanSupplier cancel) {
    record Item(Node node,double error) {}
    PriorityQueue<Item> queue=new PriorityQueue<>(Comparator.comparingDouble(Item::error).reversed());
    if(!visible(root,p,clip)) return new int[0];
    queue.add(new Item(root,p.pixelSpan(root.bounds)));
    int used=root.cost();
    List<Node> done=new ArrayList<>();
    while(!queue.isEmpty()) {
      if(cancel.getAsBoolean()) throw new java.util.concurrent.CancellationException();
      Item item=queue.remove();Node n=item.node;
      if(n.left==null || item.error<1) {done.add(n);continue;}
      boolean l=visible(n.left,p,clip),r=visible(n.right,p,clip);
      int cost=(l?n.left.cost():0)+(r?n.right.cost():0);
      if(used-n.cost()+cost>budget) {done.add(n);continue;}
      used+=cost-n.cost();
      if(l) queue.add(new Item(n.left,p.pixelSpan(n.left.bounds)));
      if(r) queue.add(new Item(n.right,p.pixelSpan(n.right.bounds)));
    }
    int[] out=new int[Math.min(used,budget)];int k=0;
    for(Node n:done) {
      int count=n.cost();
      for(int j=0;j<count && k<out.length;j++) out[k++]=ids[n.start+(int)((long)j*(n.end-n.start)/count)];
    }
    return k==out.length?out:Arrays.copyOf(out,k);
  }
  private static boolean visible(Node n,CloudRenderer.Projector p,CloudDisplay.Box clip) {
    return (clip==null || clip.intersects(n.bounds)) && p.visible(n.bounds);
  }
}
