package sgm;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.*;
import javax.swing.*;

/**
 * The 3D canvas (used by the 3D view window and the tie-point cleanup window): a CloudData drawn
 * by CloudRenderer at the screen's device resolution, camera frustums and a colour legend on top.
 * Left drag orbits, right drag (or Shift) pans, the wheel zooms towards the pointer, double-click
 * centres on a point. While moving, spatial samples and reduced resolution are used; the full picture follows
 * when the view rests. Rendering runs on its own thread; the newest request wins.
 */
final class CloudView extends JComponent {
  static final int PREVIEW_POINTS = 350_000;

  /** How to draw: colour mode and attribute, point size, shading, layers, colour range and legend title. */
  record Look(CloudRenderer.Colour colour, int attribute, int pointSize, boolean edl, boolean dense, boolean sparse, boolean cameras,
              float lo, float hi, String legend) {}

  final ThreadPoolExecutor renderer = new ThreadPoolExecutor(1,1,0,TimeUnit.MILLISECONDS,
      new ArrayBlockingQueue<>(1),GcpEditorWindow.daemon("pose-cloud-render"),new ThreadPoolExecutor.DiscardOldestPolicy());
  final ConcurrentLinkedQueue<Slot> recycled=new ConcurrentLinkedQueue<>();
  static final class Slot {
    final BufferedImage image;
    final int[] pixels; final long[] depth; final float[] shade;
    Slot(int w,int h) { image=new BufferedImage(w,h,BufferedImage.TYPE_INT_ARGB);pixels=((java.awt.image.DataBufferInt)image.getRaster().getDataBuffer()).getData();depth=new long[w*h];shade=new float[w*h]; }
  }
  Slot shown;
  volatile boolean disposed;
  int previewBudget=PREVIEW_POINTS;
  double previewScale=.75;
  double lastMillis;
  CloudDisplay display=CloudDisplay.DEFAULT;
  CloudData overview;
  Consumer<CloudRenderer.View> viewChanged=v->{};
  Runnable settled=()->{};
  Consumer<CloudDisplay.Box> rectangle=b->{};
  Consumer<Pick> selected=p->{};
  Consumer<CloudData.Cam> cameraSelected=c->{};
  Set<String> highlighted=Set.of();
  enum Tool { NAVIGATE, RECTANGLE, MEASURE, INSPECT }
  Tool tool=Tool.NAVIGATE;
  record Pick(CloudData data,int id,double[] world) {}
  final ArrayDeque<CloudRenderer.View> back=new ArrayDeque<>(),forward=new ArrayDeque<>();
  final java.util.List<double[]> measurements=new ArrayList<>();
  double[] selectedPoint;
  Rectangle dragRectangle;
  boolean rolling;
  final AtomicLong generation = new AtomicLong();
  final javax.swing.Timer settle, previewTimer;
  volatile CloudData data;
  volatile CloudRenderer.View view;
  volatile CloudRenderer.Frame frameShown;
  volatile BufferedImage image;
  Supplier<Look> look = () -> new Look(CloudRenderer.Colour.RGB, -1, 1, true, true, true, true, 0, 1, "");
  /** Hover text for the status line; null when nothing is under the pointer. */
  Consumer<String> status = s -> {};
  /** Hover text of a tie point (index), or null for the default. */
  IntFunction<String> sparseText;
  double cameraSize = 5;
  boolean moving;
  Point press;
  CloudRenderer.View pressView;
  boolean panning;

  CloudView() {
    setOpaque(true);
    setFocusable(true);
    key("alt LEFT",this::undo);key("alt RIGHT",this::redo);
    key("HOME",this::home);
    key("Q",()->{if(view!=null){remember();setView(view.withRoll(view.roll()-15),false);}});
    key("E",()->{if(view!=null){remember();setView(view.withRoll(view.roll()+15),false);}});
    previewTimer = new javax.swing.Timer(40, e -> redrawNow());
    previewTimer.setRepeats(false);
    settle = new javax.swing.Timer(180, e -> {
      moving = false;
      redraw();
      settled.run();
    });
    settle.setRepeats(false);
    MouseAdapter m = new MouseAdapter() {
      @Override
      public void mousePressed(MouseEvent e) {
        requestFocusInWindow();
        remember();
        press = e.getPoint();
        rolling=e.isControlDown() || e.isAltDown();
        pressView = view;
        panning = SwingUtilities.isRightMouseButton(e) || SwingUtilities.isMiddleMouseButton(e) || e.isShiftDown();
      }

      @Override
      public void mouseDragged(MouseEvent e) {
        if (press == null || pressView == null) return;
        int dx = e.getX() - press.x, dy = e.getY() - press.y;
        if(tool==Tool.RECTANGLE) {dragRectangle=new Rectangle(Math.min(press.x,e.getX()),Math.min(press.y,e.getY()),Math.abs(dx),Math.abs(dy));repaint();return;}
        if(tool==Tool.MEASURE)return;
        if(tool==Tool.INSPECT && !panning && !rolling)return;
        if (rolling) setView(pressView.withRoll(pressView.roll()+dx*.4),true);
        else if (panning) setView(pressView.pan(dx, dy, getHeight()), true);
        else setView(pressView.orbit(dx * .4, dy * .3), true);
      }

      @Override
      public void mouseReleased(MouseEvent e) {
        if(tool==Tool.RECTANGLE && dragRectangle!=null && dragRectangle.width>3 && dragRectangle.height>3) {
          CloudRenderer.Frame f=frameShown;
          if(f!=null) {
            double sx=f.width()/(double)Math.max(1,getWidth()),sy=f.height()/(double)Math.max(1,getHeight());
            var pr=new CloudRenderer.Projector(f.request().view(),f.width(),f.height());
            double[] t=f.request().view().target(),projected=new double[3];
            pr.project(t[0],t[1],t[2],projected);
            double x0=Double.POSITIVE_INFINITY,y0=x0,x1=Double.NEGATIVE_INFINITY,y1=x1;
            for(int x:new int[]{dragRectangle.x,dragRectangle.x+dragRectangle.width})
              for(int y:new int[]{dragRectangle.y,dragRectangle.y+dragRectangle.height}) {
                double[] w=pr.unproject(x*sx,y*sy,projected[2]);x0=Math.min(x0,w[0]);x1=Math.max(x1,w[0]);y0=Math.min(y0,w[1]);y1=Math.max(y1,w[1]);
              }
            rectangle.accept(new CloudDisplay.Box(x0,y0,Double.NEGATIVE_INFINITY,x1,y1,Double.POSITIVE_INFINITY));
          }
        }
        dragRectangle=null;repaint();
        press = null;
      }

      @Override
      public void mouseWheelMoved(MouseWheelEvent e) {
        if (view == null) return;
        if(!moving)remember();
        double factor=Math.pow(1.15,e.getPreciseWheelRotation());
        double[] p=pointAt(e.getX(),e.getY());
        CloudRenderer.View v=view.zoom(factor);
        if(p!=null) {
          double[] target=view.target();
          v=v.at(new double[]{p[0]+(target[0]-p[0])*factor,p[1]+(target[1]-p[1])*factor,p[2]+(target[2]-p[2])*factor});
        } else v=view.pan(-(e.getX()-getWidth()/2.0)*(1-factor),-(e.getY()-getHeight()/2.0)*(1-factor),getHeight()).zoom(factor);
        setView(v,true);
      }

      @Override
      public void mouseClicked(MouseEvent e) {
        if(view==null || tool==Tool.RECTANGLE || !SwingUtilities.isLeftMouseButton(e))return;
        if(tool==Tool.NAVIGATE) {
          CloudData.Cam c=cameraAt(e.getX(),e.getY());
          if(c!=null) {highlighted=Set.of(c.name());cameraSelected.accept(c);repaint();return;}
        }
        int id=pickId(e.getX(),e.getY());
        double[] p=pointAt(e.getX(),e.getY());
        if(p==null){if(tool==Tool.INSPECT)status.accept("No point under the pointer. Zoom in or increase point size to select a point.");return;}
        selectedPoint=p;
        if(tool==Tool.MEASURE) {
          if(measurements.size()==2)measurements.clear();measurements.add(p.clone());repaint();
        }
        CloudData d=(id&CloudRenderer.OVERVIEW)!=0?frameShown.request().overview():frameShown.request().data();
        id &= ~CloudRenderer.OVERVIEW;
        selected.accept(new Pick(d,id,new double[]{p[0]+d.origin[0],p[1]+d.origin[1],p[2]+d.origin[2]}));
        if(e.getClickCount()==2 && tool==Tool.NAVIGATE) {remember();setView(view.at(p),false);}
        repaint();
      }

      @Override
      public void mouseMoved(MouseEvent e) {
        hover(e.getX(), e.getY());
      }
    };
    addMouseListener(m);
    addMouseMotionListener(m);
    addMouseWheelListener(m);
    addComponentListener(new ComponentAdapter() {
      @Override
      public void componentResized(ComponentEvent e) {
        redraw();
      }
    });
  }

  /** Inspect and selection tools use a crosshair; navigation retains the normal pointer. */
  void setTool(Tool selectedTool) {
    tool=selectedTool;
    setCursor(Cursor.getPredefinedCursor(tool==Tool.NAVIGATE?Cursor.DEFAULT_CURSOR:Cursor.CROSSHAIR_CURSOR));
  }

  /** Shows new data; the home view when asked (or when there is no view yet). */
  void setData(CloudData d, boolean home) {
    boolean preserveFrame=!home && frameShown!=null && data!=null && d.sourceIndex!=null
        && d.sourceIndex==data.sourceIndex && Arrays.equals(d.origin,data.origin)
        && d.lasFiles.equals(data.lasFiles);
    generation.incrementAndGet();
    if(!preserveFrame){
      frameShown=null;image=null;
      if(shown!=null){recycle(shown);shown=null;}
    }
    data = d;
    if(!preserveFrame){selectedPoint=null;measurements.clear();}
    cameraSize = CloudRenderer.cameraSize(d);
    if (home || view == null) view = CloudRenderer.home(d);
    redraw();
  }

  void setView(CloudRenderer.View v, boolean interactive) {
    if (v == null) return;
    view = v;
    moving=interactive;
    if(interactive)settle.restart();else settle.stop();
    redraw();
    viewChanged.accept(v);
  }

  void home() {
    if (data != null) {remember();setView(CloudRenderer.home(data), false);settled.run();}
  }

  double deviceScale() {
    return getGraphicsConfiguration() == null ? 1 : getGraphicsConfiguration().getDefaultTransform().getScaleX();
  }

  void redraw() {
    if (disposed) return;
    if (moving) {
      if (!previewTimer.isRunning()) previewTimer.start();
      return;
    }
    previewTimer.stop();
    redrawNow();
  }

  private void redrawNow() {
    CloudData d=data;CloudRenderer.View v=view;
    if(disposed || d==null || v==null)return;
    long g=generation.incrementAndGet();
    double scale=deviceScale()*(moving?previewScale:1);
    int w=Math.max(1,(int)Math.round(getWidth()*scale)),h=Math.max(1,(int)Math.round(getHeight()*scale));
    Look l=look.get();CloudDisplay style=display;boolean interactive=moving;
    int budget=interactive?previewBudget:Integer.MAX_VALUE;
    CloudRenderer.Request q=new CloudRenderer.Request(d,v,w,h,l.colour(),l.attribute(),Math.max(1,(int)Math.round(l.pointSize()*scale)),l.edl(),l.dense(),l.sparse(),1,l.lo(),l.hi(),style,()->disposed || generation.get()!=g,budget,d.detailRegion!=null?overview:null);
    renderer.execute(()->{
      if(q.cancelled().getAsBoolean())return;
      Slot slot=recycled.poll();
      if(slot==null || slot.image.getWidth()!=w || slot.image.getHeight()!=h)slot=new Slot(w,h);
      final Slot out=slot;
      long started=System.nanoTime();
      try {
        CloudRenderer.Frame f=CloudRenderer.render(q,out.depth,out.pixels,out.shade);
        double elapsed=(System.nanoTime()-started)/1e6;
        SwingUtilities.invokeLater(()->{
          if(disposed || generation.get()!=g || data!=d){recycle(out);return;}
          Slot old=shown;shown=out;frameShown=f;image=out.image;lastMillis=elapsed;
          if(old!=null)recycle(old);
          if(interactive) {
            double ratio=Math.max(.5,Math.min(1.3,45/Math.max(1,elapsed)));
            previewBudget=(int)Math.max(20_000,Math.min(1_000_000,previewBudget*ratio));
            previewScale=elapsed>80?.5:elapsed<30?.85:.7;
          }
          repaint();
        });
      }catch(CancellationException ignored){
        recycle(out);
        double elapsed=(System.nanoTime()-started)/1e6;
        if(interactive && elapsed>40)SwingUtilities.invokeLater(()->{
          previewBudget=Math.max(20_000,previewBudget/2);
          previewScale=.5;
        });
      }
      catch(RuntimeException|OutOfMemoryError failure){
        SwingUtilities.invokeLater(()->{if(!disposed && generation.get()==g)status.accept("Cannot render: "+failure.getMessage());});
      }
    });
  }
  void recycle(Slot slot) {if(!disposed && recycled.size()<1)recycled.offer(slot);}
  void key(String stroke,Runnable action) {
    getInputMap(WHEN_FOCUSED).put(KeyStroke.getKeyStroke(stroke),stroke);
    getActionMap().put(stroke,new AbstractAction(){public void actionPerformed(ActionEvent e){action.run();}});
  }
  void remember() {
    if(view==null)return;
    if(back.size()>=50)back.removeFirst();back.addLast(view);forward.clear();
  }
  void undo() {if(!back.isEmpty()){forward.addLast(view);setView(back.removeLast(),false);settled.run();}}
  void redo() {if(!forward.isEmpty()){back.addLast(view);setView(forward.removeLast(),false);settled.run();}}
  int pickId(int x,int y) {
    var f=frameShown;if(f==null)return -1;
    double sx=f.width()/(double)Math.max(1,getWidth()),sy=f.height()/(double)Math.max(1,getHeight());
    return f.pick((int)Math.round(x*sx),(int)Math.round(y*sy),Math.max(1,(int)Math.round(4*Math.min(sx,sy))));
  }
  CloudData.Cam cameraAt(int x,int y) {
    var f=frameShown;
    if(f==null || !look.get().cameras())return null;
    var d=f.request().data();var pr=new CloudRenderer.Projector(f.request().view(),getWidth(),getHeight());
    double best=9;CloudData.Cam picked=null;double[] s=new double[3];
    for(var c:d.cameras) if(pr.project(c.center()[0],c.center()[1],c.center()[2],s)) {
      double distance=Math.hypot(s[0]-x,s[1]-y);
      if(distance<best){best=distance;picked=c;}
    }
    return picked;
  }

  void dispose() {
    disposed=true;generation.incrementAndGet();
    renderer.shutdownNow();recycled.clear();shown=null;
    settle.stop();previewTimer.stop();
    data = null;overview=null;
    frameShown = null;
    image = null;
  }

  /** Cloud coordinates of the point under (x, y) logical pixels, or null. */
  double[] pointAt(int x, int y) {
    CloudRenderer.Frame f = frameShown;
    CloudData d = f==null?null:f.request().data();
    if (f == null || d == null) return null;
    int id=pickId(x,y);
    if (id < 0) return null;
    if ((id & CloudRenderer.SPARSE) != 0) {
      int i = id & ~CloudRenderer.SPARSE;
      return new double[] {d.sx[i], d.sy[i], d.sz[i]};
    }
    if ((id & CloudRenderer.LIDAR) != 0) {
      int i = id & ~CloudRenderer.LIDAR;
      return new double[] {d.lx[i], d.ly[i], d.lz[i]};
    }
    if((id&CloudRenderer.OVERVIEW)!=0){d=f.request().overview();id &= ~CloudRenderer.OVERVIEW;}
    return new double[] {d.x[id], d.y[id], d.z[id]};
  }

  void hover(int x, int y) {
    CloudRenderer.Frame f = frameShown;
    CloudData d = f==null?null:f.request().data();
    if (d == null || f == null) return;
    if (look.get().cameras()) {
      CloudRenderer.Projector pr = new CloudRenderer.Projector(f.request().view(), getWidth(), getHeight());
      double[] s = new double[3];
      double best = 8;
      String text = null;
      for (CloudData.Cam c : d.cameras)
        if (pr.project(c.center()[0], c.center()[1], c.center()[2], s)) {
          double dist = Math.hypot(s[0] - x, s[1] - y);
          if (dist < best) {
            best = dist;
            text = "Camera " + c.name() + (c.flag() != null ? " - flagged: " + c.flag() : "")
                + (c.rms() != null ? String.format(Locale.ROOT, " - reprojection RMS %.2f px", c.rms()) : "");
          }
        }
      if (text != null) {
        status.accept(text);
        return;
      }
    }
    int id=pickId(x,y);
    if (id < 0) {
      status.accept(null);
      return;
    }
    if ((id & CloudRenderer.SPARSE) != 0) {
      int i = id & ~CloudRenderer.SPARSE;
      String own = sparseText == null ? null : sparseText.apply(i);
      status.accept(own != null ? own
          : String.format(Locale.ROOT, "Tie point %.3f, %.3f, %.3f - seen in %d images", d.sx[i] + d.origin[0], d.sy[i] + d.origin[1], d.sz[i] + d.origin[2], d.sobs[i]));
      return;
    }
    if ((id & CloudRenderer.LIDAR) != 0) {
      int i = id & ~CloudRenderer.LIDAR;
      status.accept(String.format(Locale.ROOT, "Lidar reference point %.3f, %.3f, %.3f", d.lx[i] + d.origin[0], d.ly[i] + d.origin[1], d.lz[i] + d.origin[2]));
      return;
    }
    if((id&CloudRenderer.OVERVIEW)!=0){d=f.request().overview();id &= ~CloudRenderer.OVERVIEW;}
    StringBuilder s = new StringBuilder(String.format(Locale.ROOT, "Point %.3f, %.3f, %.3f - class %d", d.x[id] + d.origin[0], d.y[id] + d.origin[1], d.z[id] + d.origin[2], d.cls[id] & 255));
    for (CloudData.Attribute a : d.attributes)
      if (!Float.isNaN(a.values()[id])) s.append(String.format(Locale.ROOT, " - %s %.3g", a.label().toLowerCase(Locale.ROOT), a.values()[id]));
    status.accept(s.toString());
  }

  @Override
  protected void paintComponent(Graphics g0) {
    Graphics2D g = (Graphics2D) g0.create();
    g.setColor(new Color(0x20, 0x20, 0x26));
    g.fillRect(0, 0, getWidth(), getHeight());
    CloudRenderer.Frame f = frameShown;
    BufferedImage im = image;
    if (im != null) {
      g.drawImage(im,0,0,getWidth(),getHeight(),null);
    } else {
      g.setColor(Color.LIGHT_GRAY);
      g.drawString("Loading...", 20, 30);
    }
    CloudData d = data;
    Look l = look.get();
    if (f != null && d != null && l.cameras()) cameras(g, f, d);
    if (f != null && d != null) {legend(g, f, d, l);overlays(g,f); }
    if(dragRectangle!=null){g.setColor(new Color(0xffb74d));g.draw(dragRectangle);}
    g.dispose();
  }

  void cameras(Graphics2D g, CloudRenderer.Frame f, CloudData d) {
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    CloudRenderer.Projector pr = new CloudRenderer.Projector(f.request().view(), getWidth(), getHeight());
    double[] s = new double[3];
    for (CloudData.Cam c : d.cameras) {
      double[][] fr = CloudRenderer.frustum(c, cameraSize);
      int[] px = new int[5], py = new int[5];
      boolean ok = true;
      for (int i = 0; i < 5 && ok; i++) {
        ok = pr.project(fr[i][0], fr[i][1], fr[i][2], s);
        px[i] = (int) Math.round(s[0]);
        py[i] = (int) Math.round(s[1]);
      }
      if (!ok) continue;
      g.setColor(highlighted.contains(c.name())?new Color(0xffb74d):c.flag() != null ? new Color(0xff4d4d) : new Color(0xf5f5f5));
      g.setStroke(new BasicStroke(highlighted.contains(c.name())?2.5f:c.flag() != null ? 1.6f : 1f));
      for (int i = 1; i <= 4; i++) {
        g.drawLine(px[0], py[0], px[i], py[i]);
        g.drawLine(px[i], py[i], px[i % 4 + 1], py[i % 4 + 1]);
      }
    }
  }

  void legend(Graphics2D g, CloudRenderer.Frame f, CloudData d, Look l) {
    CloudRenderer.Colour m = f.request().colour();
    g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
    int x = 12, y = getHeight() - 16;
    FontMetrics fm = g.getFontMetrics();
    if ((m == CloudRenderer.Colour.HEIGHT || m == CloudRenderer.Colour.ATTRIBUTE) && f.request().dense() && d.n > 0) {
      int w = 200;
      for (int i = 0; i < w; i++) {
        g.setColor(new Color(CloudRenderer.ramp(i / (float) (w - 1))));
        g.drawLine(x + i, y - 12, x + i, y - 2);
      }
      g.setColor(Color.WHITE);
      double off = m == CloudRenderer.Colour.HEIGHT ? d.origin[2] : 0;
      String a = m == CloudRenderer.Colour.HEIGHT ? String.format(Locale.ROOT, "%.1f", f.request().lowValue() + off) : String.format(Locale.ROOT, "%.3g", f.request().lowValue());
      String b = m == CloudRenderer.Colour.HEIGHT ? String.format(Locale.ROOT, "%.1f", f.request().highValue() + off) : String.format(Locale.ROOT, "%.3g", f.request().highValue());
      g.drawString(a, x, y - 16);
      g.drawString(b, x + w - fm.stringWidth(b), y - 16);
      g.drawString(l.legend(), x + w + 10, y - 2);
    } else if (m == CloudRenderer.Colour.CLASS && f.request().dense() && d.n > 0) {
      int[] classes = {1, 2, 3, 4, 5, 6, 7};
      String[] names = {"unclassified", "ground", "low veg.", "medium veg.", "high veg.", "building", "noise"};
      for (int i = 0; i < classes.length; i++) {
        g.setColor(new Color(CloudRenderer.classColour(classes[i])));
        g.fillRect(x, y - 10, 10, 10);
        g.setColor(Color.WHITE);
        g.drawString(names[i], x + 14, y);
        x += 24 + fm.stringWidth(names[i]);
      }
    } else if (!l.legend().isEmpty() && !f.request().dense()) {
      g.setColor(Color.WHITE);
      g.drawString(l.legend(), x, y);
    }
  }
  void overlays(Graphics2D g,CloudRenderer.Frame f) {
    var v=f.request().view();var pr=new CloudRenderer.Projector(v,getWidth(),getHeight());
    int ox=getWidth()-66,oy=62;double[] r=v.right(),u=v.up();
    String[] names={"X / E","Y / N","Z"};
    Color[] colours={new Color(0xff7777),new Color(0x80dc90),new Color(0x80bfff)};
    for(int a=0;a<3;a++) {
      int x=ox+(int)(r[a]*32),y=oy-(int)(u[a]*32);
      g.setColor(colours[a]);g.drawLine(ox,oy,x,y);g.drawString(names[a],x+3,y);
    }
    g.setColor(Color.WHITE);
    g.drawString(String.format(Locale.ROOT,"%.0f ms%s",lastMillis,moving?" · preview":""),12,20);
    if(v.ortho()) {
      double perPixel=2*v.distance()*Math.tan(Math.toRadians(v.fov()/2))/Math.max(1,getHeight());
      double raw=100*perPixel,power=Math.pow(10,Math.floor(Math.log10(raw))),units=Math.floor(raw/power)*power;
      int len=(int)(units/perPixel),y=getHeight()-70;
      g.drawLine(12,y,12+len,y);g.drawLine(12,y-3,12,y+3);g.drawLine(12+len,y-3,12+len,y+3);
      g.drawString(String.format(Locale.ROOT,"%.3g output units",units),12,y-7);
    }
    double[] a=new double[3],b=new double[3];
    if(selectedPoint!=null && pr.project(selectedPoint[0],selectedPoint[1],selectedPoint[2],a)) {
      g.setColor(Color.YELLOW);g.drawOval((int)a[0]-6,(int)a[1]-6,12,12);
    }
    if(measurements.size()==2) {
      double[] p=measurements.get(0),q=measurements.get(1);
      if(pr.project(p[0],p[1],p[2],a) && pr.project(q[0],q[1],q[2],b)) {
        g.setColor(Color.YELLOW);g.drawLine((int)a[0],(int)a[1],(int)b[0],(int)b[1]);
      }
      double xy=Math.hypot(q[0]-p[0],q[1]-p[1]),dz=q[2]-p[2];
      g.setColor(Color.YELLOW);g.drawString(String.format(Locale.ROOT,"Sampled points: distance %.4f; horizontal %.4f; ΔZ %+.4f output units",Math.hypot(xy,dz),xy,dz),12,42);
    }
  }

}
