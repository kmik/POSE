package sgm;

import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;
import javax.swing.*;
import javax.swing.border.EmptyBorder;

/** Display and inspection tools shared by the primary and comparison canvases. No project writes. */
final class CloudViewerControls {
  final CloudWindow w;
  final JPanel component=new JPanel(new BorderLayout());
  final JTabbedPane tabs=new JTabbedPane();
  final JPanel scenes=new JPanel(new BorderLayout());
  JSplitPane mainSplit;
  final CloudView comparison=new CloudView();
  final JPanel primaryPanel=new JPanel(new BorderLayout()),comparisonPanel=new JPanel(new BorderLayout());
  final JLabel comparisonTitle=new JLabel("Comparison");
  final JComboBox<String> layout=new JComboBox<>(new String[]{"Primary only","Split","Comparison only"});
  final JComboBox<String> runs=new JComboBox<>();
  final Map<String,ProjectFiles.Run> available=new LinkedHashMap<>();
  ProjectFiles.Run compareRun,displayedComparisonRun;
  CloudData compareData;
  final CloudDetailCache primaryCache=new CloudDetailCache(),comparisonCache=new CloudDetailCache();
  CloudDisplay.Box comparisonLoadingRegion;
  final ThreadPoolExecutor worker=(ThreadPoolExecutor)Executors.newFixedThreadPool(2,GcpEditorWindow.daemon("cloud-inspect"));
  final AtomicLong compareGeneration=new AtomicLong(),selectionGeneration=new AtomicLong();
  Future<?> compareTask,inspectTask,fitTask;
  boolean syncing,updating,disposed;
  final JCheckBox lockRange=new JCheckBox("Lock colour limits across runs"),autoDetail=new JCheckBox("Load local detail after navigation",true);
  final JSpinner memory=new JSpinner(new SpinnerNumberModel(512,64,8192,64));
  final JTextField colourLow=new JTextField(8),colourHigh=new JTextField(8);
  final Histogram histogram=new Histogram();
  final JComboBox<String> denseTint=new JComboBox<>(new String[]{"Original","Cyan","Orange","White"});
  final JComboBox<String> lidarTint=new JComboBox<>(new String[]{"Orange","Cyan","White","Magenta"});
  final JCheckBox comparisonDense=new JCheckBox("Comparison dense",true),comparisonSparse=new JCheckBox("Comparison tie points"),comparisonCameras=new JCheckBox("Comparison cameras",true);
  final JSlider shading=new JSlider(0,40,12);
  final JTextField classes=new JTextField(16);
  final Map<String,JTextField[]> filters=new LinkedHashMap<>();
  final JTextField[] clip=new JTextField[6];
  final JCheckBox clipOn=new JCheckBox("Enable box");
  final JComboBox<String> slice=new JComboBox<>(new String[]{"Off","X","Y","Z"});
  final JSpinner sliceCentre=new JSpinner(new SpinnerNumberModel(0.0,null,null,1.0)),sliceWidth=new JSpinner(new SpinnerNumberModel(1.0,.0001,null,.5));
  final JTextArea inspector=new JTextArea("Click a point or camera to inspect it.\nMeasurements use displayed samples.\nAll dimensions use the run's output units.",10,25);
  final DefaultListModel<String> supportModel=new DefaultListModel<>();
  final JList<String> support=new JList<>(supportModel);
  final JLabel toolInfo=new JLabel(" ");
  CloudDisplay.Box worldBox;
  Map<String,CloudDisplay.Range> activeRanges=Map.of();
  Set<Integer> activeClasses=Set.of();
  final javax.swing.Timer detailTimer;
  final Map<String,float[]> colourLimits=new HashMap<>();
  final ConcurrentHashMap<Path,CloudPointMetadata> metadata=new ConcurrentHashMap<>();
  String currentColour="RGB";
  final Set<CloudData> pendingConfidence=Collections.newSetFromMap(new IdentityHashMap<>());
  ProjectFiles.Run inspectedRun;

  CloudViewerControls(CloudWindow window) {
    w=window;
    primaryPanel.add(new JLabel("Run "+w.run.id()),BorderLayout.NORTH);primaryPanel.add(w.canvas,BorderLayout.CENTER);
    comparisonPanel.add(comparisonTitle,BorderLayout.NORTH);comparisonPanel.add(comparison,BorderLayout.CENTER);
    scenes.add(primaryPanel,BorderLayout.CENTER);
    detailTimer=new javax.swing.Timer(550,e->detail());detailTimer.setRepeats(false);
    configure(w.canvas,false);configure(comparison,true);
    comparison.look=this::comparisonLook;
    JToolBar navigation=new JToolBar();navigation.setFloatable(false);
    CloudToolIcons.addTo(navigation,t->{
      w.canvas.setTool(t);comparison.setTool(t);
      if(t==CloudView.Tool.INSPECT){tabs.setSelectedIndex(2);toolInfo.setText("Inspect: click a point. Right drag pans; wheel zooms. Values remain in the Inspect panel.");}
      else if(t==CloudView.Tool.RECTANGLE){tabs.setSelectedIndex(1);toolInfo.setText("Clip: drag a rectangle in top view, then choose Navigate to orbit the clipped region.");}
      else if(t==CloudView.Tool.MEASURE)toolInfo.setText("Measure: click two points for distance and height difference.");
      else toolInfo.setText("Navigate: drag to orbit, right drag to pan, wheel to zoom.");
      if(t==CloudView.Tool.RECTANGLE){w.ortho.setSelected(true);if(w.canvas.view!=null)w.canvas.setView(w.canvas.view.withAngles(0,90).withOrtho(true),false);}
    });
    navigation.addSeparator();
    button(navigation,"Back",w.canvas::undo);button(navigation,"Forward",w.canvas::redo);
    button(navigation,"Fit visible",this::fitVisible);
    for(String name:List.of("Front","Back side","Left","Right","Bottom"))button(navigation,name,()->preset(name));
    button(navigation,"Roll −",()->roll(-15));button(navigation,"Roll +",()->roll(15));
    button(navigation,"Clear measurement",()->{w.canvas.measurements.clear();comparison.measurements.clear();w.canvas.repaint();comparison.repaint();});
    JScrollPane navScroll=new JScrollPane(navigation,ScrollPaneConstants.VERTICAL_SCROLLBAR_NEVER,ScrollPaneConstants.HORIZONTAL_SCROLLBAR_AS_NEEDED);
    navScroll.setBorder(null);navScroll.setPreferredSize(new Dimension(600,49));
    component.add(navScroll,BorderLayout.NORTH);
    tabs.addTab("Display",displayPanel());tabs.addTab("Clip",clipPanel());tabs.addTab("Inspect",inspectPanel());tabs.addTab("Compare",comparePanel());
    tabs.setPreferredSize(new Dimension(350,700));
    JSplitPane split=new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,scenes,tabs);mainSplit=split;split.setResizeWeight(1);tabs.setMinimumSize(new Dimension(325,200));split.setDividerLocation(900);
    component.add(split,BorderLayout.CENTER);
    component.add(toolInfo,BorderLayout.SOUTH);

    for(AbstractButton b:List.of(w.edl,w.showDense,w.showSparse,w.showCameras))b.addActionListener(e->syncDisplay());
    w.pointSize.addChangeListener(e->comparison.redraw());
  }

  void configure(CloudView v,boolean other) {
    v.status=s->{if(s==null)w.describe();else w.status.setText(s);};
    v.viewChanged=view->{
      if(syncing)return;
      CloudView dest=other?w.canvas:comparison;
      if(v.data!=null && dest.data!=null){syncing=true;dest.setView(translate(view,v.data,dest.data),v.moving);syncing=false;}
      if(w.canvas.view!=null)primaryCache.navigated(w.canvas.view);
      if(comparison.view!=null)comparisonCache.navigated(comparison.view);
      if(primaryCache.zoomingOut){
        w.showCached(primaryCache.overview);
        if(compareRun==displayedComparisonRun)showCachedComparison(comparisonCache.overview);
      } else if(w.loadingRegion!=null && w.data!=null) {
        var requested=detailBox(w.data,w.canvas.view,w.canvas.getWidth(),w.canvas.getHeight(),worldBox);
        if(requested!=null && !encloses(w.loadingRegion,requested))w.cancelDetailLoad();
      }
      w.ortho.setSelected(view.ortho());
      if(!v.moving && !other)detailTimer.restart();
    };
    v.settled=()->{if(!other)detailTimer.restart();};
    v.rectangle=box->{
      if(v.data==null)return;
      CloudDisplay.Box b=box.shift(v.data.origin[0],v.data.origin[1],v.data.origin[2]);
      CloudData d=w.data;
      double[] ext=extent(d);
      setBox(new CloudDisplay.Box(b.x0(),b.y0(),ext[2],b.x1(),b.y1(),ext[5]));
      clipOn.setSelected(true);applyClip();
    };
    v.selected=p->inspect(p,other?displayedComparisonRun:w.run);
    v.cameraSelected=c->{
      ProjectFiles.Run run=other?displayedComparisonRun:w.run;if(run==null)return;
      inspectedRun=run;supportModel.clear();supportModel.addElement(c.name());
      inspector.setText("Camera "+c.name()+"\n"+(c.flag()==null?"No screening flags":c.flag())+(c.rms()==null?"":"\nReprojection RMS: "+c.rms()+" px"));
      ImageWindow.openImage(w.frame,run,c.name());
    };
  }

  static CloudRenderer.View translate(CloudRenderer.View v,CloudData from,CloudData to) {
    return v.shifted(from.origin[0]-to.origin[0],from.origin[1]-to.origin[1],from.origin[2]-to.origin[2]);
  }

  JComponent displayPanel() {
    JPanel p=column();
    p.add(new JLabel("Colour limits (blank = automatic 2–98%)"));p.add(row(colourLow,colourHigh));
    button(p,"Apply colour limits",()->{try{setColourLimits();}catch(IllegalArgumentException e){error(e);}});
    button(p,"Automatic colour limits",()->{colourLimits.remove(currentColour);w.computeRange();colourChanged();});
    p.add(lockRange);lockRange.addActionListener(e->{if(!lockRange.isSelected())colourLimits.clear();colourChanged();});p.add(histogram);
    p.add(new JLabel("Shading strength"));p.add(shading);shading.addChangeListener(e->syncDisplay());
    p.add(row(new JLabel("Dense tint"),denseTint));p.add(row(new JLabel("Lidar colour"),lidarTint));
    denseTint.addActionListener(e->syncDisplay());lidarTint.addActionListener(e->syncDisplay());
    p.add(new JSeparator());p.add(new JLabel("Visibility filters — inclusive min / max"));
    for(String key:List.of("height","confidence","views","roughness_rms")) {
      JTextField lo=new JTextField(6),hi=new JTextField(6);filters.put(key,new JTextField[]{lo,hi});
      p.add(pairRow(key.equals("height")?"Height":CloudData.label(key),lo,hi));
    }
    p.add(new JLabel("Class IDs (comma separated)"));p.add(row(classes));
    button(p,"Apply visibility filters",this::applyFilters);
    button(p,"Reset filters",()->{for(var pair:filters.values())for(var field:pair)field.setText("");classes.setText("");applyFilters();});
    p.add(new JSeparator());p.add(row(new JLabel("Viewer allowance (MiB)"),memory));
    memory.setToolTipText("Allowance for dense samples and replacement buffers; comparison shares it. Image buffers and JVM overhead are additional.");
    memory.addChangeListener(e->{w.load();if(compareRun!=null)loadComparison();});
    p.add(autoDetail);button(p,"Load detail in this view",()->{detail(true);});
    button(p,"Restore overview",()->{autoDetail.setSelected(false);if(primaryCache.overview!=null)w.showCached(primaryCache.overview);else w.load();if(compareRun==displayedComparisonRun)showCachedComparison(comparisonCache.overview);});
    p.add(new JLabel("Detail follows the view; clipped areas load at finer density."));
    return scroll(p);
  }

  JComponent clipPanel() {
    JPanel p=column();p.add(new JLabel("Box coordinates in output units"));p.add(clipOn);
    for(int axis=0;axis<3;axis++) {
      clip[axis]=new JTextField(9);clip[axis+3]=new JTextField(9);
      p.add(pairRow(new String[]{"X / East","Y / North","Z / Height"}[axis],clip[axis],clip[axis+3]));
    }
    button(p,"Apply box",this::applyClip);
    button(p,"Box from cloud bounds",()->{if(w.data!=null){double[] b=extent(w.data);setBox(new CloudDisplay.Box(b[0],b[1],b[2],b[3],b[4],b[5]));}});
    p.add(new JLabel("Rectangle tool draws an XY box in top view."));
    p.add(new JSeparator());p.add(row(new JLabel("Slice axis"),slice));
    p.add(row(new JLabel("Centre"),sliceCentre));p.add(row(new JLabel("Thickness"),sliceWidth));
    slice.addActionListener(e->applyClip());sliceCentre.addChangeListener(e->applyClip());sliceWidth.addChangeListener(e->applyClip());clipOn.addActionListener(e->applyClip());
    button(p,"Clear box and slice",()->{updating=true;clipOn.setSelected(false);slice.setSelectedIndex(0);updating=false;worldBox=null;syncDisplay();detailTimer.restart();});
    p.add(new JLabel("The same world box and slice apply to both runs."));
    return scroll(p);
  }

  JComponent inspectPanel() {
    JPanel p=column();inspector.setEditable(false);inspector.setLineWrap(true);inspector.setWrapStyleWord(true);
    p.add(new JScrollPane(inspector));p.add(new JLabel("Supporting cameras / selected photograph"));
    support.setVisibleRowCount(6);p.add(new JScrollPane(support));
    button(p,"Open selected photograph",()->{String name=support.getSelectedValue();if(name!=null && inspectedRun!=null)ImageWindow.openImage(w.frame,inspectedRun,name);});
    support.addListSelectionListener(e->{if(!e.getValueIsAdjusting()){
      String name=support.getSelectedValue();if(name!=null){w.canvas.highlighted=Set.of(name);comparison.highlighted=Set.of(name);w.canvas.repaint();comparison.repaint();}
    }});
    p.add(new JLabel("Ctrl/Alt + drag: roll. Q / E: roll by 15°."));
    p.add(new JLabel("Alt + Left / Right: view history. Home: reset."));
    return scroll(p);
  }

  JComponent comparePanel() {
    JPanel p=column();runs.addItem("Choose a run");
    Path root=w.run.dir().getParent()==null?null:w.run.dir().getParent().getParent();
    if(root!=null)for(ProjectFiles.Run r:ProjectFiles.runs(root))if(!r.dir().equals(w.run.dir())){available.put(r.id(),r);runs.addItem(r.id());}
    p.add(new JLabel("Compare in the same coordinate frame"));p.add(runs);
    runs.addActionListener(e->{if(updating)return;compareRun=available.get(runs.getSelectedItem());if(compareRun!=null)loadComparison();});
    button(p,"Choose run folder…",()->{
      JFileChooser chooser=new JFileChooser(w.run.dir().getParent().toFile());chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      if(chooser.showOpenDialog(w.frame)==JFileChooser.APPROVE_OPTION){Path path=chooser.getSelectedFile().toPath();
        if(CloudData.lasFiles(path).isEmpty()){toolInfo.setText("Choose a run folder containing 07_results/dense.las or LAS tiles.");return;}
        compareRun=new ProjectFiles.Run(path.getFileName().toString(),path,null,"",List.of(),null,List.of(),List.of(),List.of());loadComparison();
      }
    });
    p.add(row(new JLabel("Layout"),layout));layout.addActionListener(e->layout());
    for(JCheckBox b:List.of(comparisonDense,comparisonSparse,comparisonCameras)){p.add(b);b.addActionListener(e->comparison.redraw());}
    button(p,"Remove comparison",()->{compareGeneration.incrementAndGet();if(compareTask!=null)compareTask.cancel(true);worker.purge();compareRun=null;displayedComparisonRun=null;compareData=null;comparisonCache.clear();comparison.overview=null;comparison.generation.incrementAndGet();comparison.data=null;comparison.frameShown=null;comparison.image=null;layout.setSelectedIndex(0);w.load();});
    p.add(new JLabel("Views, clipping and colour limits stay synchronized."));
    p.add(new JLabel("Only compare runs with compatible CRS and units."));
    return scroll(p);
  }

  long pointBudget() {
    long mib=((Number)memory.getValue()).longValue();
    long allowance=Math.min(mib<<20,Runtime.getRuntime().maxMemory()/3);
    int attributes=w.data==null?12:w.data.attributes.size()+2;
    return CloudData.budget(CloudWindow.BUDGETS[Math.max(0,w.budget.getSelectedIndex())],allowance/(compareRun==null?1:2),attributes);
  }

  void adjustLayout() {if(mainSplit!=null)mainSplit.setDividerLocation(Math.max(250,component.getWidth()-365));}
  void dataChanged() {
    if(w.data==null)return;
    primaryCache.accept(w.data);
    primaryCache.navigated(w.canvas.view);
    w.canvas.overview=primaryCache.overview;
    if(clip[0].getText().isBlank()) {
      double[] b=extent(w.data);setBox(new CloudDisplay.Box(b[0],b[1],b[2],b[3],b[4],b[5]));
      updating=true;sliceCentre.setValue((b[2]+b[5])/2);updating=false;
    }
    colourChanged();syncDisplay();syncViews();
  }

  void colourChanged() {
    if(w.updatingColour || w.data==null)return;
    String name=String.valueOf(w.colour.getSelectedItem());currentColour=name;
    float[] limits=colourLimits.get(name);
    if(limits!=null) {w.lo=limits[0];w.hi=limits[1];}
    double offset=w.mode()==CloudRenderer.Colour.HEIGHT?w.data.origin[2]:0;
    colourLow.setText(String.format(Locale.ROOT,"%.9g",w.lo+offset));colourHigh.setText(String.format(Locale.ROOT,"%.9g",w.hi+offset));
    float[] values=w.mode()==CloudRenderer.Colour.HEIGHT?w.data.z:w.attributeIndex()<0?null:w.data.attributes.get(w.attributeIndex()).values();
    histogram.values(values,w.data.n,w.lo,w.hi);
    if(lockRange.isSelected() && limits==null)colourLimits.put(name,new float[]{w.lo,w.hi});
    if(compareData!=null && (name.equals(CloudData.label("confidence"))||name.equals(CloudData.label("views"))))loadFilterConfidence(compareData,displayedComparisonRun,true);
    syncDisplay();
  }

  void setColourLimits() {
    String lowText=colourLow.getText().trim(),highText=colourHigh.getText().trim();
    float oldLow=w.lo,oldHigh=w.hi;
    w.computeRange();float autoLow=w.lo,autoHigh=w.hi;w.lo=oldLow;w.hi=oldHigh;
    if(lowText.isEmpty() && highText.isEmpty()){colourLimits.remove(currentColour);w.lo=autoLow;w.hi=autoHigh;colourChanged();return;}
    double offset=w.mode()==CloudRenderer.Colour.HEIGHT && w.data!=null?w.data.origin[2]:0;
    double lo=lowText.isEmpty()?autoLow+offset:Double.parseDouble(lowText),hi=highText.isEmpty()?autoHigh+offset:Double.parseDouble(highText);
    if(!Double.isFinite(lo)||!Double.isFinite(hi)||hi<=lo || !Float.isFinite((float)(lo-offset)) || !Float.isFinite((float)(hi-offset)))
      throw new IllegalArgumentException("Colour maximum must be greater than minimum and within the display range");
    w.lo=(float)(lo-offset);w.hi=(float)(hi-offset);
    colourLimits.put(currentColour,new float[]{w.lo,w.hi});colourChanged();
  }

  void applyFilters() {
    try {
      Map<String,CloudDisplay.Range> ranges=new LinkedHashMap<>();
      for(var e:filters.entrySet()) {
        String a=e.getValue()[0].getText().trim(),b=e.getValue()[1].getText().trim();
        if(a.isEmpty() && b.isEmpty())continue;
        double lo=a.isEmpty()?Double.NEGATIVE_INFINITY:Double.parseDouble(a),hi=b.isEmpty()?Double.POSITIVE_INFINITY:Double.parseDouble(b);
        if(Double.isNaN(lo)||Double.isNaN(hi)||lo>hi)throw new IllegalArgumentException("Invalid limits for "+e.getKey());
        ranges.put(e.getKey(),new CloudDisplay.Range(lo,hi));
      }
      Set<Integer> c=new HashSet<>();if(!classes.getText().isBlank())for(String v:classes.getText().split(",")){int id=Integer.parseInt(v.trim());if(id<0||id>255)throw new IllegalArgumentException("Class IDs must be 0–255");c.add(id);}
      activeRanges=Map.copyOf(ranges);activeClasses=Set.copyOf(c);syncDisplay();
      boolean need=ranges.containsKey("confidence")||ranges.containsKey("views");
      if(need)loadFilterConfidence(w.data,w.run,false);
      if(need && compareData!=null)loadFilterConfidence(compareData,displayedComparisonRun,true);
      toolInfo.setText("Visibility filters affect displayed points only; missing attribute values are hidden.");
    }catch(IllegalArgumentException e){error(e);}
  }

  void loadFilterConfidence(CloudData d,ProjectFiles.Run run,boolean other) {
    if(d==null || d.attribute("confidence")!=null || !pendingConfidence.add(d))return;
    worker.submit(()->{
      try {
        // Populate privately; publication of a new attribute list is performed on the EDT.
        CloudData temp=new CloudData();temp.n=d.n;temp.originalIds=d.originalIds;temp.lasFiles.addAll(d.lasFiles);
        boolean ok=temp.loadConfidence(run.dir(),null);
        SwingUtilities.invokeLater(()->{pendingConfidence.remove(d);if(disposed || (other?compareData:w.data)!=d)return;
          if(ok){for(var a:temp.attributes)if(d.attribute(a.name())==null)d.attributes.add(a);if(!other)w.colourChanged();syncDisplay();}else toolInfo.setText("Confidence is unavailable for this cloud (tiled LAS provenance is not mapped).");
        });
      }catch(IOException e){SwingUtilities.invokeLater(()->{pendingConfidence.remove(d);if(!disposed)toolInfo.setText(e.getMessage());});}
    });
  }

  void loadActiveConfidence() {
    if(activeRanges.containsKey("confidence")||activeRanges.containsKey("views"))loadFilterConfidence(w.data,w.run,false);
  }
  void setBox(CloudDisplay.Box b) {
    double[] v={b.x0(),b.y0(),b.z0(),b.x1(),b.y1(),b.z1()};
    for(int i=0;i<6;i++)clip[i].setText(String.format(Locale.ROOT,"%.9f",v[i]));
  }

  void applyClip() {
    if(updating || w.data==null)return;
    try {
      CloudDisplay.Box b=null;
      if(clipOn.isSelected()) {double[] v=new double[6];for(int i=0;i<6;i++){v[i]=Double.parseDouble(clip[i].getText());if(!Double.isFinite(v[i]))throw new IllegalArgumentException("Box coordinates must be finite");}b=new CloudDisplay.Box(v[0],v[1],v[2],v[3],v[4],v[5]);}
      int axis=slice.getSelectedIndex()-1;
      if(axis>=0) {
        double[] ext=b==null?extent(w.data):new double[]{b.x0(),b.y0(),b.z0(),b.x1(),b.y1(),b.z1()};
        double centre=((Number)sliceCentre.getValue()).doubleValue(),half=((Number)sliceWidth.getValue()).doubleValue()/2;
        double a=centre-half,bound=centre+half;
        if(b!=null){a=Math.max(a,ext[axis]);bound=Math.min(bound,ext[axis+3]);}
        if(a>bound)throw new IllegalArgumentException("Slice lies outside the box");
        ext[axis]=a;ext[axis+3]=bound;
        b=new CloudDisplay.Box(ext[0],ext[1],ext[2],ext[3],ext[4],ext[5]);
      }
      worldBox=b;syncDisplay();detailTimer.restart();
    }catch(IllegalArgumentException e){error(e);}
  }

  CloudDisplay style(CloudData d) {
    CloudDisplay.Box b=worldBox==null||d==null?null:worldBox.shift(-d.origin[0],-d.origin[1],-d.origin[2]);
    Integer tint=switch(denseTint.getSelectedIndex()){case 1->0x4dd0e1;case 2->0xff9800;case 3->0xeeeeee;default->null;};
    int lidar=switch(lidarTint.getSelectedIndex()){case 1->0x4dd0e1;case 2->0xeeeeee;case 3->0xff66dd;default->0xff9800;};
    return new CloudDisplay(b,activeRanges,activeClasses,tint,lidar,shading.getValue());
  }

  void syncDisplay() {w.canvas.display=style(w.data);comparison.display=style(compareData);w.canvas.redraw();comparison.redraw();}

  CloudView.Look comparisonLook() {
    CloudView.Look l=w.look();int index=-1;
    if(compareData!=null && w.data!=null && l.attribute()>=0 && l.attribute()<w.data.attributes.size()) {
      String name=w.data.attributes.get(l.attribute()).name();for(int i=0;i<compareData.attributes.size();i++)if(compareData.attributes.get(i).name().equals(name))index=i;
    }
    float low=l.lo(),high=l.hi();if(l.colour()==CloudRenderer.Colour.HEIGHT && w.data!=null && compareData!=null){low+=(float)(w.data.origin[2]-compareData.origin[2]);high+=(float)(w.data.origin[2]-compareData.origin[2]);}
    return new CloudView.Look(l.colour(),index,l.pointSize(),l.edl(),comparisonDense.isSelected(),comparisonSparse.isSelected(),comparisonCameras.isSelected(),low,high,index<0 && l.colour()==CloudRenderer.Colour.ATTRIBUTE?"Attribute unavailable":l.legend());
  }

  void syncViews() {
    if(w.canvas.view!=null && w.data!=null && compareData!=null){syncing=true;comparison.setView(translate(w.canvas.view,w.data,compareData),false);syncing=false;}
  }

  void loadComparison() {loadComparison(null);}
  void loadComparison(CloudDisplay.Box region) {
    if(compareRun==null || disposed)return;
    if(region==null){comparisonCache.clearDetail();showCachedComparison(comparisonCache.overview);}
    long token=compareGeneration.incrementAndGet();if(compareTask!=null)compareTask.cancel(true);
    comparisonLoadingRegion=region;
    ProjectFiles.Run run=compareRun;long budget=region==null?pointBudget():CloudDetailCache.detailBudget(pointBudget());toolInfo.setText("Loading comparison "+run.id()+"…");
    compareTask=worker.submit(()->{
      try {
        CloudSourceIndex index=compareData==null?null:compareData.sourceIndex;
        CloudData d=CloudData.load(run.dir(),budget,null,region,index);
        SwingUtilities.invokeLater(()->{
          if(disposed || compareGeneration.get()!=token)return;
          if(!compatible(w.data,d)){compareRun=null;displayedComparisonRun=null;compareData=null;comparisonCache.clear();comparison.overview=null;comparisonLoadingRegion=null;layout.setSelectedIndex(0);toolInfo.setText("Comparison refused: the runs declare different CRS or coordinate multipliers.");return;}
          comparisonLoadingRegion=null;
          displayedComparisonRun=run;compareData=d;comparisonCache.accept(d);comparison.overview=comparisonCache.overview;comparison.setData(d,false);comparisonTitle.setText("Run "+run.id());syncViews();comparisonCache.navigated(comparison.view);syncDisplay();if(region==null)layout.setSelectedIndex(1);
          toolInfo.setText("Comparison shares the view and world clipping. Verify local frames have the same origin and axes.");
          if(w.data!=null && w.data.n>pointBudget())w.load();
          if(w.mode()==CloudRenderer.Colour.ATTRIBUTE || activeRanges.containsKey("confidence") || activeRanges.containsKey("views"))loadFilterConfidence(d,run,true);
        });
      }catch(Exception|OutOfMemoryError e){SwingUtilities.invokeLater(()->{if(!disposed && compareGeneration.get()==token)toolInfo.setText("Cannot load comparison: "+e.getMessage());});}
    });
  }

  void prepareOverviewLoad() {
    primaryCache.clearDetail();
    if(primaryCache.overview!=null && w.data!=primaryCache.overview)w.showCached(primaryCache.overview);
  }

  void cancelComparisonDetail() {
    if(comparisonLoadingRegion==null)return;
    comparisonLoadingRegion=null;compareGeneration.incrementAndGet();
    if(compareTask!=null)compareTask.cancel(true);
    worker.purge();
  }

  void showCachedComparison(CloudData cached) {
    cancelComparisonDetail();
    if(cached==null || cached==compareData)return;
    compareData=cached;comparison.setData(cached,false);comparison.overview=comparisonCache.overview;
    syncViews();syncDisplay();
  }

  static boolean compatible(CloudData a,CloudData b) {
    return a==null || Objects.equals(a.crs,b.crs) && Double.compare(a.multiplier,b.multiplier)==0;
  }

  void layout() {
    scenes.removeAll();int mode=layout.getSelectedIndex();
    if(mode==1 && compareData!=null){JSplitPane split=new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,primaryPanel,comparisonPanel);split.setResizeWeight(.5);scenes.add(split);SwingUtilities.invokeLater(()->split.setDividerLocation(.5));}
    else scenes.add(mode==2 && compareData!=null?comparisonPanel:primaryPanel);
    scenes.revalidate();scenes.repaint();syncViews();
  }

  void detail() {detail(false);}
  void detail(boolean force) {
    CloudData d=w.data;if(disposed || d==null || d.total==0 || !force && !autoDetail.isSelected())return;
    CloudDisplay.Box b=detailBox(d,w.canvas.view,w.canvas.getWidth(),w.canvas.getHeight(),worldBox);
    if(b==null)return;
    CloudData cached=primaryCache.cached(b,force);
    if(cached!=null)w.showCached(cached);
    else {
      if(w.loading!=null && !w.loading.isDone()){
        if(w.loadingRegion==null){detailTimer.restart();return;}
        if(encloses(w.loadingRegion,b))return;
      }
      w.load(worldBox==null?CloudDetailCache.padded(b):b);
    }
    if(compareData!=null && compareRun!=null && compareRun==displayedComparisonRun){
      var box=b.shift(d.origin[0]-compareData.origin[0],d.origin[1]-compareData.origin[1],d.origin[2]-compareData.origin[2]);
      CloudData other=comparisonCache.cached(box,force);
      if(other!=null)showCachedComparison(other);
      else if(compareTask==null || compareTask.isDone() || comparisonLoadingRegion!=null && !encloses(comparisonLoadingRegion,box))
        loadComparison(worldBox==null?CloudDetailCache.padded(box):box);
    }
  }

  static boolean encloses(CloudDisplay.Box a,CloudDisplay.Box b) {
    return a.x0()<=b.x0() && a.y0()<=b.y0() && a.z0()<=b.z0() && a.x1()>=b.x1() && a.y1()>=b.y1() && a.z1()>=b.z1();
  }

  static CloudDisplay.Box detailBox(CloudData d,CloudRenderer.View v,int width,int height,CloudDisplay.Box worldClip) {
    if(v==null)return null;
    if(worldClip!=null)return worldClip.shift(-d.origin[0],-d.origin[1],-d.origin[2]);
    // A world-space volume around the view target; preserves depth while spending samples locally.
    double half=v.distance()*Math.tan(Math.toRadians(v.fov()/2))*1.3,aspect=Math.max(1,width)/(double)Math.max(1,height);
    double[] target=v.target();double r=half*Math.max(1,aspect);
    return new CloudDisplay.Box(target[0]-r,target[1]-r,target[2]-r,target[0]+r,target[1]+r,target[2]+r);
  }

  void inspect(CloudView.Pick pick,ProjectFiles.Run run) {
    if(run==null)return;
    if(w.canvas.tool==CloudView.Tool.INSPECT || comparison.tool==CloudView.Tool.INSPECT)tabs.setSelectedIndex(2);
    long token=selectionGeneration.incrementAndGet();if(inspectTask!=null)inspectTask.cancel(true);worker.purge();
    inspectedRun=run;supportModel.clear();CloudData d=pick.data();int id=pick.id();double[] p=pick.world();
    String text=String.format(Locale.ROOT,"Run %s\nX %.6f\nY %.6f\nZ %.6f\n",run.id(),p[0],p[1],p[2]);
    boolean dense=(id&(CloudRenderer.SPARSE|CloudRenderer.LIDAR))==0;
    if(!dense){inspector.setText(text+((id&CloudRenderer.SPARSE)!=0?"Tie point\nObserved in "+d.sobs[id&~CloudRenderer.SPARSE]+" images":"Lidar reference point"));inspector.setCaretPosition(0);return;}
    long original=d.pointId(id);int rgb=d.rgb[id];
    StringBuilder info=new StringBuilder(text).append("Source point ID ").append(original).append("\nClass ").append(d.cls[id]&255)
        .append("\nRGB ").append(rgb>>>16&255).append(", ").append(rgb>>>8&255).append(", ").append(rgb&255);
    for(var a:d.attributes)if(Float.isFinite(a.values()[id]))info.append(String.format(Locale.ROOT,"\n%s %.6g",a.label(),a.values()[id]));
    inspector.setText(info+"\nReading provenance…");inspector.setCaretPosition(0);
    Path file=run.dir().resolve("07_results/point-confidence.bin");
    if(d.lasFiles.size()!=1 || !Files.isRegularFile(file)){inspector.setText(info+"\nProvenance unavailable for this cloud.");return;}
    inspectTask=worker.submit(()->{
      try {
        var result=metadata.computeIfAbsent(file,CloudPointMetadata::new).find(original);
        RunImages images=RunImages.load(run.dir());
        SwingUtilities.invokeLater(()->{
          if(disposed || selectionGeneration.get()!=token)return;
          if(result==null){inspector.setText(info+"\nProvenance record missing.");return;}
          info.append(String.format(Locale.ROOT,"\nConfidence %.6g\nSupporting views %d\nIndependent pairs %d\nRelative stereo sigma %.6g",result.confidence(),result.views(),result.pairs(),result.sigma()));
          inspector.setText(info.toString());Set<String> names=new HashSet<>();BitSet ids=result.cameras();
          if(ids!=null)for(var im:images.images)if(ids.get(im.id())){supportModel.addElement(im.name());names.add(im.name());}
          w.canvas.highlighted=Set.copyOf(names);comparison.highlighted=Set.copyOf(names);w.canvas.repaint();comparison.repaint();
          if(supportModel.isEmpty())toolInfo.setText("Supporting camera names are unavailable for this point.");
        });
      }catch(IOException e){SwingUtilities.invokeLater(()->{if(!disposed && selectionGeneration.get()==token)inspector.setText(info+"\nProvenance: "+e.getMessage());});}
    });
  }

  void preset(String name) {
    if(w.canvas.view==null)return;
    w.canvas.remember();double yaw=switch(name){case "Back side"->180;case "Left"->90;case "Right"->270;default->0;};
    w.canvas.setView(w.canvas.view.withAngles(yaw,name.equals("Bottom")?270:0),false);
  }
  void roll(double angle) {if(w.canvas.view!=null){w.canvas.remember();w.canvas.setView(w.canvas.view.withRoll(w.canvas.view.roll()+angle),false);}}

  void fitVisible() {
    CloudData d=w.data;if(d==null)return;
    CloudDisplay style=w.canvas.display;CloudView.Look look=w.look();CloudRenderer.View v=w.canvas.view;
    if(fitTask!=null)fitTask.cancel(true);
    toolInfo.setText("Fitting visible layers…");
    fitTask=worker.submit(()->{
      double[] lo={Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY},hi={Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
      for(int i=0;i<d.n;i++){
        if((i&4095)==0 && Thread.currentThread().isInterrupted())return;
        if(look.dense() && style.dense(d,i))grow(lo,hi,d.x[i],d.y[i],d.z[i]);
      }
      for(int i=0;i<d.sx.length;i++)if(look.sparse() && style.position(d.sx[i],d.sy[i],d.sz[i]) && (d.shidden==null||!d.shidden[i]))grow(lo,hi,d.sx[i],d.sy[i],d.sz[i]);
      if(d.showLidar)for(int i=0;i<d.lx.length;i++)if(style.position(d.lx[i],d.ly[i],d.lz[i]))grow(lo,hi,d.lx[i],d.ly[i],d.lz[i]);
      if(look.cameras())for(var c:d.cameras)grow(lo,hi,c.center()[0],c.center()[1],c.center()[2]);
      SwingUtilities.invokeLater(()->{
        if(disposed || w.data!=d || w.canvas.view!=v)return;
        if(!Double.isFinite(lo[0])){toolInfo.setText("No visible points to fit.");return;}
        double radius=Math.max(.001,Math.sqrt(Math.pow(hi[0]-lo[0],2)+Math.pow(hi[1]-lo[1],2)+Math.pow(hi[2]-lo[2],2))/2);
        double[] target={(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2};
        double aspect=w.canvas.getWidth()/(double)Math.max(1,w.canvas.getHeight());
        double angle=Math.atan(Math.tan(Math.toRadians(v.fov()/2))*Math.min(1,aspect));
        w.canvas.remember();w.canvas.setView(new CloudRenderer.View(target,v.yaw(),v.pitch(),radius/Math.sin(angle)*1.1,v.fov(),v.ortho(),v.roll()),false);
        toolInfo.setText("Fitted visible layers.");
      });
    });
  }
  static void grow(double[] lo,double[] hi,double x,double y,double z){lo[0]=Math.min(lo[0],x);lo[1]=Math.min(lo[1],y);lo[2]=Math.min(lo[2],z);hi[0]=Math.max(hi[0],x);hi[1]=Math.max(hi[1],y);hi[2]=Math.max(hi[2],z);}
  static double[] extent(CloudData d) {
    if(d.sourceIndex!=null && d.sourceIndex.root!=null){var b=d.sourceIndex.root.bounds;return new double[]{b.x0(),b.y0(),b.z0(),b.x1(),b.y1(),b.z1()};}
    double[] lo={Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY},hi={Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
    for(int i=0;i<d.n;i++)grow(lo,hi,d.x[i]+d.origin[0],d.y[i]+d.origin[1],d.z[i]+d.origin[2]);
    if(d.n==0)for(int i=0;i<d.sx.length;i++)grow(lo,hi,d.sx[i]+d.origin[0],d.sy[i]+d.origin[1],d.sz[i]+d.origin[2]);
    if(!Double.isFinite(lo[0])){lo=d.origin.clone();hi=d.origin.clone();for(int i=0;i<3;i++){lo[i]-=1;hi[i]+=1;}}
    return new double[]{lo[0],lo[1],lo[2],hi[0],hi[1],hi[2]};
  }

  void dispose() {disposed=true;detailTimer.stop();compareGeneration.incrementAndGet();selectionGeneration.incrementAndGet();worker.shutdownNow();comparison.dispose();primaryCache.clear();comparisonCache.clear();}
  void error(Exception e){toolInfo.setText(e.getMessage()==null?"Invalid value":e.getMessage());}
  static JPanel column(){JPanel p=new JPanel();p.setLayout(new BoxLayout(p,BoxLayout.Y_AXIS));p.setBorder(new EmptyBorder(8,8,8,8));return p;}
  static JPanel pairRow(String label,JComponent low,JComponent high) {
    JPanel p=new JPanel(new BorderLayout(5,0));JLabel title=new JLabel(label);title.setPreferredSize(new Dimension(90,24));p.add(title,BorderLayout.WEST);
    JPanel fields=new JPanel(new GridLayout(1,2,5,0));fields.add(low);fields.add(high);p.add(fields);p.setMaximumSize(new Dimension(Integer.MAX_VALUE,26));p.setAlignmentX(Component.LEFT_ALIGNMENT);return p;
  }
  static JPanel row(Component... components){JPanel p=new JPanel(new FlowLayout(FlowLayout.LEFT,4,3));for(Component c:components)p.add(c);p.setAlignmentX(Component.LEFT_ALIGNMENT);p.setMaximumSize(new Dimension(Integer.MAX_VALUE,p.getPreferredSize().height));return p;}
  static JScrollPane scroll(JComponent p){JScrollPane s=new JScrollPane(p);s.setBorder(null);s.getVerticalScrollBar().setUnitIncrement(20);return s;}
  static void button(JComponent p,String label,Runnable action){JButton b=new JButton(label);b.setAlignmentX(Component.LEFT_ALIGNMENT);b.addActionListener(e->action.run());p.add(b);}

  static final class Histogram extends JComponent {
    int[] bins=new int[48];int max=1;String caption="Colour distribution of loaded samples";
    Histogram(){setPreferredSize(new Dimension(280,90));setMaximumSize(new Dimension(600,100));}
    void values(float[] v,int n,float lo,float hi){Arrays.fill(bins,0);max=1;if(v!=null){int stride=Math.max(1,n/20_000);for(int i=0;i<n;i+=stride)if(Float.isFinite(v[i])){int b=hi>lo?(int)((v[i]-lo)/(hi-lo)*bins.length):bins.length/2;b=Math.max(0,Math.min(bins.length-1,b));max=Math.max(max,++bins[b]);}}repaint();}
    @Override protected void paintComponent(Graphics g){g.setColor(new Color(0x596579));g.drawString(caption,4,15);for(int i=0;i<bins.length;i++){g.setColor(new Color(CloudRenderer.ramp(i/(float)bins.length)));int h=(getHeight()-23)*bins[i]/max;g.fillRect(i*getWidth()/bins.length,getHeight()-h,Math.max(1,getWidth()/bins.length),h);}}
  }
}
