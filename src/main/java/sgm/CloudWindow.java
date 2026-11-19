package sgm;

import java.awt.*;
import java.awt.event.*;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;
import javax.swing.*;
import javax.swing.border.EmptyBorder;

/**
 * The 3D view of a run (one window per run): dense cloud, sparse tie points and cameras, coloured
 * by RGB, height, classification or a per-point attribute (the canvas is CloudView). Nothing is
 * written.
 */
final class CloudWindow {
  static final Map<Path, CloudWindow> OPEN = new HashMap<>();
  static final long[] BUDGETS = {1_000_000, 3_000_000, 6_000_000, 12_000_000};

  final ProjectFiles.Run run;
  final JFrame frame;
  final CloudView canvas = new CloudView();
  final JLabel status = new JLabel("Loading...");
  final JComboBox<String> colour = new JComboBox<>();
  final JSpinner pointSize = new JSpinner(new SpinnerNumberModel(1, 1, 5, 1));
  final JCheckBox edl = new JCheckBox("Shading", true), showDense = new JCheckBox("Dense", true), showSparse = new JCheckBox("Tie points", false),
      showCameras = new JCheckBox("Cameras", true), ortho = new JCheckBox("Orthographic", false), showLidar = new JCheckBox("Lidar", false);
  final JComboBox<String> budget = new JComboBox<>(new String[] {"1 M points", "3 M points", "6 M points", "12 M points"});
  final ThreadPoolExecutor loader = new ThreadPoolExecutor(1,1,0,TimeUnit.MILLISECONDS,new LinkedBlockingQueue<>(),GcpEditorWindow.daemon("pose-cloud-load"));
  final java.util.concurrent.atomic.AtomicLong loadGeneration=new java.util.concurrent.atomic.AtomicLong();
  Future<?> loading,confidenceTask,lidarTask;
  CloudDisplay.Box loadingRegion;
  boolean closed,updatingColour,confidenceLoading;
  CloudViewerControls controls;
  volatile CloudData data;
  volatile float lo, hi;

  static void open(Component parent, ProjectFiles.Run run) {
    CloudWindow w = OPEN.get(run.dir());
    if (w != null && w.frame.isDisplayable()) {
      w.frame.toFront();
      return;
    }
    boolean any = !CloudData.lasFiles(run.dir()).isEmpty() || Files.isRegularFile(run.dir().resolve("04_geometry/sparse.csv"))
        || Files.isRegularFile(run.dir().resolve("sparse.csv")) || Files.isRegularFile(run.dir().resolve("04_geometry/cameras.csv"));
    if (!any) {
      JOptionPane.showMessageDialog(parent, "The run has no points or cameras yet (they come with the georeference step).", "3D view", JOptionPane.INFORMATION_MESSAGE);
      return;
    }
    OPEN.put(run.dir(), new CloudWindow(run, parent));
  }

  CloudWindow(ProjectFiles.Run run, Component parent) {
    this.run = run;
    frame = new JFrame("3D view - run " + run.id());
    GuiSupport.brand(frame);
    frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    frame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosed(WindowEvent e) {
        closed=true;loadGeneration.incrementAndGet();
        if(controls!=null)controls.dispose();
        canvas.dispose();
        loader.shutdownNow();
        data = null;
        OPEN.remove(run.dir());
      }
    });
    budget.setSelectedIndex(2);
    canvas.look = this::look;
    canvas.status = s -> {
      if (s == null) describe();
      else status.setText(s);
    };

    JToolBar bar = new JToolBar();
    bar.setFloatable(false);
    bar.add(new JLabel(" Colour "));
    bar.add(colour);
    bar.addSeparator();
    bar.add(new JLabel("Point size "));
    ((JSpinner.DefaultEditor) pointSize.getEditor()).getTextField().setColumns(2);
    bar.add(pointSize);
    bar.addSeparator();
    for (JCheckBox b : List.of(edl, showDense, showSparse, showCameras, ortho, showLidar)) bar.add(b);
    bar.addSeparator();
    JButton home = new JButton("Home"), top = new JButton("Top"), side = new JButton("Side");
    home.setToolTipText("Everything in view, from the south at 55 degrees");
    top.setToolTipText("Straight down, north up, orthographic (like a map)");
    side.setToolTipText("From the south, level (heights and layers)");
    for (JButton b : List.of(home, top, side)) bar.add(b);
    bar.addSeparator();
    bar.add(budget);
    edl.setToolTipText("Eye-dome lighting: darkens points behind nearer ones, so edges and relief stand out");
    showSparse.setToolTipText("Tie points of the alignment, coloured by the number of images that see them (cyan few, red many)");
    showLidar.setToolTipText("The lidar reference points of a run made with --lidar-reference, within the cloud's extent");
    showLidar.setEnabled(!CloudData.lidarFiles(run.dir()).isEmpty());
    showLidar.addActionListener(e -> lidarToggled());
    showCameras.setToolTipText("Camera positions and viewing directions; red: flagged by camera screening. Hover for the name.");
    budget.setToolTipText("Maximum displayed dense samples; also capped by the viewer memory allowance. Local detail loads replace the overview.");

    home.addActionListener(e -> {ortho.setSelected(false);canvas.home();});
    top.addActionListener(e -> {
      ortho.setSelected(true);
      if (canvas.view != null) canvas.setView(canvas.view.withAngles(0, 90).withOrtho(true), false);
    });
    side.addActionListener(e -> {
      ortho.setSelected(false);
      if (canvas.view != null) canvas.setView(canvas.view.withAngles(0, 0).withOrtho(false), false);
    });
    ortho.addActionListener(e -> {
      if (canvas.view != null) canvas.setView(canvas.view.withOrtho(ortho.isSelected()), false);
    });
    colour.addActionListener(e -> colourChanged());
    pointSize.addChangeListener(e -> canvas.redraw());
    for (JCheckBox b : List.of(edl, showDense, showSparse, showCameras)) b.addActionListener(e -> canvas.redraw());
    budget.addActionListener(e -> {load();if(controls!=null && controls.compareRun!=null)controls.loadComparison();});

    canvas.setPreferredSize(new Dimension(1100, 720));
    status.setBorder(new EmptyBorder(3, 6, 3, 6));
    JPanel content = new JPanel(new BorderLayout());
    content.add(bar, BorderLayout.NORTH);
    controls=new CloudViewerControls(this);
    content.add(controls.component, BorderLayout.CENTER);
    content.add(status, BorderLayout.SOUTH);
    frame.setContentPane(content);
    frame.pack();
    frame.setSize(1450,900);
    frame.setLocationRelativeTo(parent);
    frame.setVisible(true);
    controls.adjustLayout();
    load();
  }

  CloudView.Look look() {
    return new CloudView.Look(mode(), attributeIndex(), ((Number) pointSize.getValue()).intValue(), edl.isSelected(), showDense.isSelected(),
        showSparse.isSelected(), showCameras.isSelected(), lo, hi, String.valueOf(colour.getSelectedItem()));
  }

  void load() {load(null);}
  void load(CloudDisplay.Box region) {
    if(closed)return;
    if(region==null && controls!=null)controls.prepareOverviewLoad();
    long token=loadGeneration.incrementAndGet();
    if(loading!=null)loading.cancel(true);
    if(confidenceTask!=null)confidenceTask.cancel(true);
    if(lidarTask!=null)lidarTask.cancel(true);
    confidenceLoading=false;loader.purge();
    loadingRegion=region;
    status.setText(region==null?"Loading overview...":"Loading local detail...");
    long max=controls==null?BUDGETS[Math.max(0,budget.getSelectedIndex())]:controls.pointBudget();
    if(region!=null)max=CloudDetailCache.detailBudget(max);
    final long sampleBudget=max;
    CloudSourceIndex index=data==null?null:data.sourceIndex;
    loading=loader.submit(()->{
      try {
        CloudData d=CloudData.load(run.dir(),sampleBudget,msg->SwingUtilities.invokeLater(()->{
          if(!closed && token==loadGeneration.get())status.setText(msg);
        }),region,index);
        SwingUtilities.invokeLater(()->{if(!closed && token==loadGeneration.get()){loadingRegion=null;loaded(d);}});
      }catch(Exception|OutOfMemoryError e) {
        if(Thread.currentThread().isInterrupted())return;
        SwingUtilities.invokeLater(()->{
          if(!closed && token==loadGeneration.get() && !(e instanceof java.io.InterruptedIOException) && !(e instanceof CancellationException)){
            status.setText("Could not read the run: "+e.getMessage());GuiSupport.error(frame,"3D view",e);
          }
        });
      }
    });
  }

  void cancelDetailLoad() {
    if(loadingRegion==null)return;
    loadGeneration.incrementAndGet();loadingRegion=null;
    if(loading!=null)loading.cancel(true);
    loading=null;loader.purge();
  }

  void showCached(CloudData cached) {
    cancelDetailLoad();
    if(cached!=null && cached!=data){
      if(confidenceTask!=null)confidenceTask.cancel(true);
      if(lidarTask!=null)lidarTask.cancel(true);
      loader.purge();loaded(cached);
    }
  }

  /** First time on: read the lidar points within the cloud (same budget as the dense cloud), then draw them. */
  void lidarToggled() {
    CloudData d = data;
    if (d == null) return;
    if (!showLidar.isSelected() || d.lx.length > 0) {
      d.showLidar = showLidar.isSelected();
      controls.syncDisplay();
      canvas.redraw();
      return;
    }
    long max = controls.pointBudget();
    status.setText("Reading the lidar reference...");
    if(lidarTask!=null && !lidarTask.isDone())return;
    lidarTask=loader.submit(() -> {
      try {
        CloudData temp=new CloudData();temp.n=d.n;temp.x=d.x;temp.y=d.y;temp.sx=d.sx;temp.sy=d.sy;System.arraycopy(d.origin,0,temp.origin,0,3);
        temp.readLidar(CloudData.lidarFiles(run.dir()), max, s -> SwingUtilities.invokeLater(() -> {if(!closed && data==d)status.setText(s);}));
        SwingUtilities.invokeLater(() -> {
          if(closed || data!=d)return;
          d.lx=temp.lx;d.ly=temp.ly;d.lz=temp.lz;d.lidarInExtent=temp.lidarInExtent;
          d.showLidar = showLidar.isSelected();
          canvas.redraw();
          describe();
        });
      } catch (Exception | OutOfMemoryError e) {
        if(Thread.currentThread().isInterrupted())return;
        SwingUtilities.invokeLater(() -> {
          if(closed || data!=d)return;
          showLidar.setSelected(false);
          status.setText("Could not read the lidar reference: " + e);
        });
      }
    });
  }

  void loaded(CloudData d) {
    boolean first = data == null;
    data = d;
    confidenceLoading=false;
    updatingColour=true;
    String selected = (String) colour.getSelectedItem();
    colour.removeAllItems();
    if (d.total > 0) {
      colour.addItem("RGB");
      colour.addItem("Height");
      colour.addItem("Classification");
      for (CloudData.Attribute a : d.attributes) colour.addItem(a.label());
      if (Files.isRegularFile(run.dir().resolve("07_results/point-confidence.bin")) && d.lasFiles.size() == 1) {
        colour.addItem(CloudData.label("views"));
        colour.addItem(CloudData.label("confidence"));
      }
    } else colour.addItem("Height");
    if (selected != null) {for(int i=0;i<colour.getItemCount();i++)if(selected.equals(colour.getItemAt(i)))colour.setSelectedIndex(i); }
    updatingColour=false;
    if (d.n == 0 && d.total==0) {
      showDense.setSelected(false);
      showDense.setEnabled(false);
      showSparse.setSelected(true);
    }
    showDense.setEnabled(d.total>0);
    computeRange();
    canvas.setData(d, first);
    controls.dataChanged();
    colourChanged();
    controls.loadActiveConfidence();
    describe();
    if (showLidar.isSelected()) lidarToggled();
  }

  void describe() {
    CloudData d = data;
    if (d == null) return;
    String s = d.n > 0
        ? String.format(Locale.ROOT, "%,d of %,d dense points (every %d.)", d.n, d.total, d.stride)
        : "No dense cloud yet";
    if(d.detailRegion!=null)s=String.format(Locale.ROOT,"Local detail: %,d of %,d points in area (%,d total)",d.n,d.regionTotal,d.total);
    s += String.format(Locale.ROOT, "; %,d tie points; %d cameras", d.sx.length, d.cameras.size());
    long flagged = d.cameras.stream().filter(c -> c.flag() != null).count();
    if (flagged > 0) s += " (" + flagged + " flagged, red)";
    if (d.lx.length > 0) s += String.format(Locale.ROOT, "; %,d of %,d lidar points", d.lx.length, d.lidarInExtent);
    if (d.crs != null) s += "; " + d.crs;
    status.setText(s + ".   Drag: orbit; Ctrl+drag / Q,E: roll; right drag: pan; wheel: zoom.");
  }

  /** The colour range of the chosen mode (2% and 98% quantiles), loading confidence on request. */
  void colourChanged() {
    if(updatingColour || closed)return;
    CloudData d = data;
    String c = (String) colour.getSelectedItem();
    if (d == null || c == null) return;
    String views = CloudData.label("views"), confidence = CloudData.label("confidence");
    if ((c.equals(views) || c.equals(confidence)) && d.attributes.stream().noneMatch(a -> a.name().equals("views"))) {
      if(confidenceLoading)return;
      confidenceLoading=true;
      status.setText("Reading point confidence...");
      confidenceTask=loader.submit(() -> {
        try {
          CloudData temp=new CloudData();temp.n=d.n;temp.originalIds=d.originalIds;temp.lasFiles.addAll(d.lasFiles);
          temp.loadConfidence(run.dir(), s -> SwingUtilities.invokeLater(() -> {if(!closed && data==d)status.setText(s);}));
          SwingUtilities.invokeLater(() -> {
            if(closed || data!=d)return;
            for(var a:temp.attributes)if(d.attribute(a.name())==null)d.attributes.add(a);
            confidenceLoading=false;
            describe();
            colourChanged();
          });
        } catch (Exception e) {
          if(Thread.currentThread().isInterrupted())return;
          SwingUtilities.invokeLater(() -> {if(!closed && data==d){confidenceLoading=false;GuiSupport.error(frame, "3D view", e);}});
        }
      });
      return;
    }
    computeRange();
    if(controls!=null)controls.colourChanged();
    canvas.redraw();
  }

  void computeRange() {
    CloudData d = data;
    String c = (String) colour.getSelectedItem();
    if (d == null || c == null) return;
    float[] values = null;
    if (c.equals("Height")) values = d.n > 0 ? d.z : d.sz;
    else for (CloudData.Attribute a : d.attributes) if (a.label().equals(c)) values = a.values();
    if (values != null) {
      int count = values == d.sz ? d.sz.length : d.n;
      lo = CloudData.quantile(values, count, .02);
      hi = CloudData.quantile(values, count, .98);
    }
  }

  CloudRenderer.Colour mode() {
    String c = (String) colour.getSelectedItem();
    if (c == null || c.equals("RGB")) return CloudRenderer.Colour.RGB;
    if (c.equals("Height")) return CloudRenderer.Colour.HEIGHT;
    if (c.equals("Classification")) return CloudRenderer.Colour.CLASS;
    return CloudRenderer.Colour.ATTRIBUTE;
  }

  int attributeIndex() {
    String c = (String) colour.getSelectedItem();
    CloudData d = data;
    if (d == null) return -1;
    for (int i = 0; i < d.attributes.size(); i++) if (d.attributes.get(i).label().equals(c)) return i;
    return -1;
  }
}
