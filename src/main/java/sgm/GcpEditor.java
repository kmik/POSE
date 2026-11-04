package sgm;

import com.sun.net.httpserver.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;

/**
 * Marker editor for ground control points: {@code photogrammetry gcp-edit -o PROJECT [--run ID]
 * [--gcp-port N]}. A local web page (http://127.0.0.1:PORT) lists the GCPs and, per GCP, the
 * images where it is predicted; markers are dragged/clicked to the exact spot at full resolution,
 * pinned or rejected. Every change is saved to the run's 04_geometry/gcp/markers.json; once a GCP
 * has two pins its other predictions follow the triangulated point and each pin shows its
 * residual. "Done" stops the editor; then run the gcp step again.
 */
final class GcpEditor {
  private GcpEditor() {}

  static int serve(Main.Command c) throws IOException {
    Path root = Main.projectRoot(c.rest());
    String id = c.runId() == null ? Main.latestRun(root) : c.runId();
    Path run = root.resolve("runs").resolve(id);
    if (!PipelineState.exists(run)) throw new IllegalArgumentException("Run " + id + " has no pipeline state");
    Path file = GroundControl.markersFile(run);
    if (!Files.isRegularFile(file))
      throw new IllegalArgumentException(
          "No markers yet: run the gcp step first (photogrammetry gcp -o " + root + " ... --gcp GCPS.csv)");
    nu.pattern.OpenCV.loadLocally();
    PipelineState state = PipelineState.load(run);
    Options options = Options.parse(Main.continued(c));
    Options alignment = options.alignmentOptions();
    alignment.alignmentCalibration = true;
    List<ImageFrame> frames = state.readFrames(alignment);
    GroundControl.Markers markers = GroundControl.load(file);
    boolean georeferenced = state.string("crs") != null;
    Editor editor = new Editor(frames, markers, file, georeferenced);
    HttpServer server = HttpServer.create(new InetSocketAddress(InetAddress.getLoopbackAddress(), options.gcpPort), 0);
    Object done = new Object();
    server.createContext("/", ex -> editor.handle(ex, done));
    server.start();
    String url = "http://127.0.0.1:" + options.gcpPort + "/";
    System.out.println("GCP marker editor for run " + id + ": " + url + "  (press Done in the page to finish)");
    try {
      if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE))
        Desktop.getDesktop().browse(URI.create(url));
    } catch (Exception e) {
      // No desktop browser: the URL is printed above.
    }
    synchronized (done) {
      try {
        done.wait();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }
    server.stop(0);
    for (ImageFrame f : frames) f.close();
    String ready = GroundControl.readiness(markers, georeferenced);
    System.out.println(
        "Markers saved to "
            + file
            + (ready == null
                ? ". Next: photogrammetry gcp -o " + root
                : ". Still needed: " + ready + "."));
    return 0;
  }

  static final class Editor {
    final List<ImageFrame> frames;
    final GroundControl.Markers markers;
    final Path file;
    final boolean georeferenced;
    // Decoded photographs, least recently used first.
    final LinkedHashMap<Integer, BufferedImage> cache = new LinkedHashMap<>(8, .75f, true);

    Editor(List<ImageFrame> frames, GroundControl.Markers markers, Path file, boolean georeferenced) {
      this.frames = frames;
      this.markers = markers;
      this.file = file;
      this.georeferenced = georeferenced;
    }

    void handle(HttpExchange ex, Object done) throws IOException {
      try {
        String path = ex.getRequestURI().getPath();
        Map<String, String> q = query(ex.getRequestURI().getRawQuery());
        switch (path) {
          case "/" -> send(ex, 200, "text/html; charset=utf-8", PAGE.getBytes(StandardCharsets.UTF_8));
          case "/api/state" -> json(ex, state());
          case "/api/image" -> image(ex, q);
          case "/api/marker" -> {
            var body = JsonRead.object(JsonRead.parse(new String(ex.getRequestBody().readAllBytes(), StandardCharsets.UTF_8)));
            json(ex, update(body));
          }
          case "/api/done" -> {
            json(ex, Json.object("saved", true));
            synchronized (done) {
              done.notifyAll();
            }
          }
          default -> send(ex, 404, "text/plain", "not found".getBytes(StandardCharsets.UTF_8));
        }
      } catch (RuntimeException | IOException e) {
        send(ex, 500, "text/plain; charset=utf-8", String.valueOf(e).getBytes(StandardCharsets.UTF_8));
      }
    }

    synchronized Map<String, Object> state() {
      List<Object> images = new ArrayList<>(), gcps = new ArrayList<>(), list = new ArrayList<>();
      for (ImageFrame f : frames)
        images.add(Json.object("id", f.id, "name", f.path.getFileName().toString(), "width", f.sourceWidth, "height", f.sourceHeight, "registered", f.camera.registered));
      for (GroundControl.Point3 p : markers.gcps) {
        double[] where = GroundControl.triangulate(markers, p.id, frames);
        gcps.add(Json.object("id", p.id, "type", p.check ? "check" : "control", "xyz", p.xyz, "pins", markers.pins(p.id), "triangulated", where));
      }
      for (GroundControl.Marker k : markers.markers) {
        double[] where = GroundControl.triangulate(markers, k.gcp, frames);
        double residual = k.pinned() && where != null && markers.pins(k.gcp) >= 2 ? GroundControl.residual(frames.get(k.image), k, where) : Double.NaN;
        list.add(
            Json.object(
                "gcp", k.gcp, "image", k.image, "u", k.u, "v", k.v, "status", k.status,
                "predicted_u", Double.isFinite(k.predictedU) ? k.predictedU : null,
                "predicted_v", Double.isFinite(k.predictedV) ? k.predictedV : null,
                "residual", Double.isFinite(residual) ? residual : null));
      }
      String ready = GroundControl.readiness(markers, georeferenced);
      return Json.object("images", images, "gcps", gcps, "markers", list, "georeferenced", georeferenced, "ready", ready == null, "needed", ready);
    }

    synchronized Map<String, Object> update(Map<String, Object> body) throws IOException {
      String gcp = String.valueOf(body.get("gcp"));
      int image = ((Number) body.get("image")).intValue();
      String status = String.valueOf(body.get("status"));
      if (markers.gcp(gcp) == null || image < 0 || image >= frames.size()) throw new IllegalArgumentException("unknown GCP or image");
      if (!Set.of("pinned", "predicted", "rejected").contains(status)) throw new IllegalArgumentException("bad status");
      GroundControl.Marker k = markers.marker(gcp, image);
      if (k == null) {
        k = new GroundControl.Marker(gcp, image);
        markers.markers.add(k);
      }
      k.status = status;
      if (body.get("u") instanceof Number u && body.get("v") instanceof Number v) {
        k.u = u.doubleValue();
        k.v = v.doubleValue();
      }
      if (status.equals("predicted")) {
        k.u = k.predictedU;
        k.v = k.predictedV;
      }
      GroundControl.predict(markers, frames, georeferenced);
      GroundControl.save(markers, file, frames);
      return state();
    }

    void image(HttpExchange ex, Map<String, String> q) throws IOException {
      int id = Integer.parseInt(q.get("id"));
      BufferedImage source = photo(id);
      int out = Integer.parseInt(q.getOrDefault("out", "512"));
      BufferedImage result;
      if (q.containsKey("cx")) {
        // Square window of 'size' source pixels around (cx, cy), drawn at out x out.
        double cx = Double.parseDouble(q.get("cx")), cy = Double.parseDouble(q.get("cy"));
        int size = Integer.parseInt(q.getOrDefault("size", "512"));
        result = new BufferedImage(out, out, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = result.createGraphics();
        g.setColor(Color.DARK_GRAY);
        g.fillRect(0, 0, out, out);
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, size > out ? RenderingHints.VALUE_INTERPOLATION_BILINEAR : RenderingHints.VALUE_INTERPOLATION_NEAREST_NEIGHBOR);
        int x0 = (int) Math.round(cx - size / 2.), y0 = (int) Math.round(cy - size / 2.);
        g.drawImage(source, 0, 0, out, out, x0, y0, x0 + size, y0 + size, null);
        g.dispose();
      } else {
        // Whole photograph, longest side 'out'.
        double s = out / (double) Math.max(source.getWidth(), source.getHeight());
        result = new BufferedImage(Math.max(1, (int) (source.getWidth() * s)), Math.max(1, (int) (source.getHeight() * s)), BufferedImage.TYPE_INT_RGB);
        Graphics2D g = result.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g.drawImage(source, 0, 0, result.getWidth(), result.getHeight(), null);
        g.dispose();
      }
      ByteArrayOutputStream bytes = new ByteArrayOutputStream();
      ImageIO.write(result, "jpg", bytes);
      send(ex, 200, "image/jpeg", bytes.toByteArray());
    }

    synchronized BufferedImage photo(int id) throws IOException {
      BufferedImage image = cache.get(id);
      if (image != null) return image;
      image = ImageIO.read(frames.get(id).path.toFile());
      if (image == null) throw new IOException("Cannot decode " + frames.get(id).path);
      cache.put(id, image);
      while (cache.size() > 6) cache.remove(cache.keySet().iterator().next());
      return image;
    }

    private static Map<String, String> query(String raw) {
      Map<String, String> out = new HashMap<>();
      if (raw == null) return out;
      for (String part : raw.split("&")) {
        int eq = part.indexOf('=');
        if (eq > 0) out.put(URLDecoder.decode(part.substring(0, eq), StandardCharsets.UTF_8), URLDecoder.decode(part.substring(eq + 1), StandardCharsets.UTF_8));
      }
      return out;
    }

    private static void json(HttpExchange ex, Object value) throws IOException {
      send(ex, 200, "application/json; charset=utf-8", Json.string(value).getBytes(StandardCharsets.UTF_8));
    }

    private static void send(HttpExchange ex, int code, String type, byte[] body) throws IOException {
      ex.getResponseHeaders().set("Content-Type", type);
      ex.getResponseHeaders().set("Cache-Control", "no-store");
      ex.sendResponseHeaders(code, body.length);
      try (OutputStream out = ex.getResponseBody()) {
        out.write(body);
      }
    }
  }

  static final String PAGE =
      """
<!doctype html><html><head><meta charset="utf-8"><title>GCP markers</title>
<style>
body{margin:0;font:14px system-ui,sans-serif;display:flex;height:100vh;color:#1f2937}
#side{width:260px;border-right:1px solid #ddd;overflow:auto;padding:10px;box-sizing:border-box}
#main{flex:1;overflow:auto;padding:10px}
.gcp{padding:6px 8px;border-radius:6px;cursor:pointer;margin-bottom:4px;display:flex;justify-content:space-between}
.gcp:hover{background:#f3f4f6}.gcp.sel{background:#dbeafe}
.small{color:#6b7280;font-size:12px}
#grid{display:flex;flex-wrap:wrap;gap:10px}
.card{width:200px;border:1px solid #ddd;border-radius:6px;overflow:hidden;cursor:pointer;position:relative}
.card img{width:200px;height:200px;display:block}
.card .cap{padding:4px 6px;font-size:12px}
.pinned{border-color:#16a34a}.rejected{opacity:.45}
.cross{position:absolute;left:94px;top:94px;width:12px;height:12px;border:2px solid #ef4444;border-radius:50%;pointer-events:none}
#view{display:none;position:fixed;inset:0;background:rgba(0,0,0,.85);color:#fff;z-index:9;flex-direction:column;align-items:center;justify-content:center}
#view canvas{background:#222;cursor:crosshair}
button{font:inherit;padding:5px 10px;margin:3px;border-radius:5px;border:1px solid #9ca3af;background:#fff;cursor:pointer}
#bar{margin:8px}#done{width:100%;margin-top:10px;background:#16a34a;color:#fff;border:0;padding:8px}
</style></head><body>
<div id="side"><b>Ground control points</b><div id="ready" class="small"></div><div id="list"></div>
<button id="done">Done</button><div class="small" style="margin-top:8px">Click an image to open it. Click in the opened image to place the marker (pinned).
Keys: arrows = previous/next image, + / - zoom, R reject, U unpin, Esc close.</div></div>
<div id="main"><div id="title"></div><div id="grid"></div><div id="others"></div></div>
<div id="view"><div id="vtitle"></div><canvas id="cv" width="800" height="800"></canvas>
<div id="bar"><button id="zin">Zoom in</button><button id="zout">Zoom out</button><button id="rej">Reject</button><button id="unpin">Unpin</button>
<button id="prev">Previous</button><button id="next">Next</button><button id="close">Close</button></div><div id="vinfo" class="small" style="color:#ddd"></div></div>
<script>
let S=null,gcp=null,cards=[],cur=-1,size=400,center=null,img=new Image();
const $=id=>document.getElementById(id);
async function load(){S=await (await fetch('api/state')).json();if(!gcp&&S.gcps.length)gcp=S.gcps[0].id;render();}
function markersOf(g){return S.markers.filter(m=>m.gcp===g&&Number.isFinite(m.u));}
function render(){
 $('ready').textContent=S.ready?'Ready for the gcp step.':'Still needed: '+S.needed;
 $('list').innerHTML='';
 for(const g of S.gcps){const d=document.createElement('div');d.className='gcp'+(g.id===gcp?' sel':'');
  const n=S.markers.filter(m=>m.gcp===g.id).length;d.innerHTML=`<span>${g.id} <span class="small">${g.type}</span></span><span class="small">${g.pins} pinned / ${n}</span>`;
  d.onclick=()=>{gcp=g.id;render();};$('list').appendChild(d);}
 cards=markersOf(gcp);$('title').innerHTML=`<h3>${gcp}</h3>`;$('grid').innerHTML='';
 cards.forEach((m,i)=>{const im=S.images[m.image],c=document.createElement('div');c.className='card '+m.status;
  c.innerHTML=`<img src="api/image?id=${m.image}&cx=${m.u}&cy=${m.v}&size=200&out=200&t=${Date.now()}"><div class="cross"></div><div class="cap">${im.name}<br>${m.status}${m.residual!=null?', '+m.residual.toFixed(2)+' px':''}</div>`;
  c.onclick=()=>open(i);$('grid').appendChild(c);});
 const seen=new Set(cards.map(m=>m.image)),rest=S.images.filter(im=>im.registered&&!seen.has(im.id));
 $('others').innerHTML=rest.length?'<h4>Other images (no prediction): click to place manually</h4>':'';
 for(const im of rest){const b=document.createElement('button');b.textContent=im.name;b.onclick=()=>openManual(im.id);$('others').appendChild(b);}
}
function open(i){cur=i;const m=cards[i];center={x:m.u,y:m.v,image:m.image};size=400;show();}
function openManual(id){const im=S.images[id];cur=-1;center={x:im.width/2,y:im.height/2,image:id};size=Math.max(im.width,im.height);show();}
function show(){$('view').style.display='flex';const im=S.images[center.image];
 const m=S.markers.find(k=>k.gcp===gcp&&k.image===center.image);
 $('vtitle').textContent=`${gcp} in ${im.name} - ${m?m.status:'no marker'}${m&&m.residual!=null?', residual '+m.residual.toFixed(2)+' px':''}`;
 img=new Image();img.onload=draw;img.src=`api/image?id=${center.image}&cx=${center.x}&cy=${center.y}&size=${Math.round(size)}&out=800`;
 $('vinfo').textContent=`window ${Math.round(size)} px (${(800/size).toFixed(2)}x)`;}
function draw(){const c=$('cv').getContext('2d');c.drawImage(img,0,0);
 const m=S.markers.find(k=>k.gcp===gcp&&k.image===center.image);if(!m)return;
 const s=800/size,toX=u=>(u-center.x)*s+400,toY=v=>(v-center.y)*s+400;
 if(m.predicted_u!=null){c.strokeStyle='#facc15';c.lineWidth=1.5;c.beginPath();c.arc(toX(m.predicted_u),toY(m.predicted_v),10,0,7);c.stroke();}
 if(Number.isFinite(m.u)){c.strokeStyle=m.status==='pinned'?'#22c55e':'#ef4444';c.lineWidth=2;const x=toX(m.u),y=toY(m.v);
  c.beginPath();c.moveTo(x-14,y);c.lineTo(x-4,y);c.moveTo(x+4,y);c.lineTo(x+14,y);c.moveTo(x,y-14);c.lineTo(x,y-4);c.moveTo(x,y+4);c.lineTo(x,y+14);c.stroke();}}
async function post(status,u,v){S=await (await fetch('api/marker',{method:'POST',body:JSON.stringify({gcp,image:center.image,status,u,v})})).json();cards=markersOf(gcp);render();show();}
$('cv').onclick=e=>{const r=e.target.getBoundingClientRect(),s=800/size;const u=center.x+(e.clientX-r.left-400)/s,v=center.y+(e.clientY-r.top-400)/s;
 if(size>1200){center={x:u,y:v,image:center.image};size=400;show();return;} post('pinned',u,v);};
$('zin').onclick=()=>{size=Math.max(50,size/2);show();};$('zout').onclick=()=>{size=Math.min(8000,size*2);show();};
$('rej').onclick=()=>post('rejected');$('unpin').onclick=()=>post('predicted');
$('prev').onclick=()=>{if(cards.length){open((cur-1+cards.length)%cards.length);}};$('next').onclick=()=>{if(cards.length){open((cur+1)%cards.length);}};
$('close').onclick=()=>{$('view').style.display='none';};
document.onkeydown=e=>{if($('view').style.display!=='flex')return;const k=e.key;
 if(k==='Escape')$('close').onclick();else if(k==='ArrowRight')$('next').onclick();else if(k==='ArrowLeft')$('prev').onclick();
 else if(k==='+')$('zin').onclick();else if(k==='-')$('zout').onclick();else if(k==='r'||k==='R')post('rejected');else if(k==='u'||k==='U')post('predicted');};
$('done').onclick=async()=>{await fetch('api/done');document.body.innerHTML='<p style="padding:20px">Markers saved. You can close this tab and run the gcp step.</p>';};
load();
</script></body></html>
""";
}
