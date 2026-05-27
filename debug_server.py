"""
Web-Debug-Server für den Wave Rover
=====================================
Startet einen Flask-Webserver im Hintergrund.
Im Browser öffnen:  http://192.168.4.1:5000

Bietet:
  /              → Live-Dashboard mit allen Infos
  /stream/main   → MJPEG-Stream: Kamerabild mit Overlays
  /stream/mask   → MJPEG-Stream: Grün-Erkennungsmaske
  /status        → JSON: aktueller Zustand (für Dashboard)
"""

import io
import threading
import time
import logging
from typing import Optional, Dict, Any

import cv2
import numpy as np

try:
    from flask import Flask, Response, jsonify, render_template_string
    _FLASK_OK = True
except ImportError:
    _FLASK_OK = False

logger = logging.getLogger(__name__)

# ── HTML-Dashboard ─────────────────────────────────────────────────────────────
_HTML = r"""
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Wave Rover Debug</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0d1117;color:#c9d1d9;font-family:'Courier New',monospace;height:100vh;overflow:hidden}
  header{background:#161b22;padding:10px 18px;border-bottom:1px solid #30363d;
         display:flex;align-items:center;gap:12px}
  header h1{font-size:1.1rem;color:#58a6ff}
  #badge{padding:3px 10px;border-radius:20px;font-size:.8rem;font-weight:700;
         background:#21262d;transition:background .25s,color .25s}
  #fps-hdr{margin-left:auto;color:#8b949e;font-size:.78rem}

  .layout{display:grid;grid-template-columns:230px 1fr;gap:10px;
          padding:10px;height:calc(100vh - 47px)}

  /* ── Sidebar ── */
  .sidebar{background:#161b22;border-radius:8px;padding:12px;
           display:flex;flex-direction:column;gap:8px;overflow-y:auto}
  .grp h3{font-size:.65rem;color:#8b949e;text-transform:uppercase;
          letter-spacing:1.2px;margin-bottom:5px;padding-bottom:3px;
          border-bottom:1px solid #21262d}
  .row{display:flex;justify-content:space-between;align-items:center;
       padding:2px 0;font-size:.82rem}
  .lbl{color:#8b949e}
  .val{font-weight:700;color:#c9d1d9}
  .good{color:#3fb950}.warn{color:#d29922}.bad{color:#f85149}

  /* Offset-Balken */
  .bar-wrap{background:#0d1117;border-radius:3px;height:8px;
            position:relative;margin:3px 0 6px}
  .bar-mid{position:absolute;left:50%;top:0;width:1px;height:100%;background:#30363d}
  .bar-fill{position:absolute;top:0;height:100%;border-radius:3px;
            background:#3fb950;transition:left .1s,width .1s,background .1s}

  /* Heading-Arc */
  .arc-wrap{display:flex;justify-content:center;margin:4px 0}
  #arc-canvas{border-radius:50%}

  .hint{font-size:.65rem;color:#484f58;text-align:center;margin-top:auto;padding-top:8px}

  /* ── Streams ── */
  .streams{display:flex;flex-direction:column;gap:8px;overflow:hidden}
  .sbox{background:#161b22;border-radius:8px;overflow:hidden;
        flex:1;display:flex;flex-direction:column;min-height:0}
  .slbl{font-size:.65rem;color:#8b949e;padding:5px 10px;background:#0d1117;
        text-transform:uppercase;letter-spacing:1px}
  .sbox img{width:100%;height:calc(100% - 24px);object-fit:contain;display:block}
</style>
</head>
<body>
<header>
  <h1>🤖 Wave Rover — Live Debug</h1>
  <span id="badge">––</span>
  <span id="fps-hdr">FPS: –</span>
</header>

<div class="layout">
 <div class="sidebar">

  <div class="grp">
   <h3>Fahrt</h3>
   <div class="row"><span class="lbl">Zustand</span>  <span class="val" id="v-state">–</span></div>
   <div class="row"><span class="lbl">Basis-Speed</span><span class="val" id="v-speed">–</span></div>
   <div class="row"><span class="lbl">Eff. Speed</span> <span class="val" id="v-espeed">–</span></div>
  </div>

  <div class="grp">
   <h3>Pfad</h3>
   <div class="row"><span class="lbl">Erkannt</span><span class="val" id="v-found">–</span></div>
   <div class="row"><span class="lbl">Offset</span> <span class="val" id="v-offset">–</span></div>
   <div class="bar-wrap">
     <div class="bar-mid"></div>
     <div class="bar-fill" id="bar-fill" style="left:50%;width:0"></div>
   </div>
   <div class="row"><span class="lbl">Fläche</span><span class="val" id="v-area">–</span></div>
   <div class="row"><span class="lbl">Toter Ber.</span><span class="val" id="v-dz">–</span></div>
  </div>

  <div class="grp">
   <h3>Knick-Erkennung</h3>
   <div class="row"><span class="lbl">Winkel</span>    <span class="val" id="v-bend">–</span></div>
   <div class="row"><span class="lbl">Richtung</span>  <span class="val" id="v-bdir">–</span></div>
   <div class="row"><span class="lbl">Speed-Fakt.</span><span class="val" id="v-sf">–</span></div>
   <div class="row"><span class="lbl">Scharfer Knick</span><span class="val" id="v-sharp">–</span></div>
  </div>

  <div class="grp">
   <h3>Heading (Rückwärts-Schutz)</h3>
   <div class="row"><span class="lbl">Gedreht</span><span class="val" id="v-hdeg">–</span></div>
   <div class="row"><span class="lbl">Limit</span>  <span class="val" id="v-hlim">–</span></div>
   <div class="arc-wrap"><canvas id="arc-canvas" width="100" height="60"></canvas></div>
  </div>

  <div class="grp">
   <h3>System</h3>
   <div class="row"><span class="lbl">FPS</span>       <span class="val" id="v-fps">–</span></div>
   <div class="row"><span class="lbl">Frames</span>    <span class="val" id="v-frames">–</span></div>
   <div class="row"><span class="lbl">Uptime</span>    <span class="val" id="v-uptime">–</span></div>
  </div>

  <p class="hint">↻ 300 ms Polling · MJPEG-Stream</p>
 </div>

 <div class="streams">
  <div class="sbox">
   <div class="slbl">📷 Kamera mit Overlays</div>
   <img src="/stream/main" alt="Kamera-Stream">
  </div>
  <div class="sbox">
   <div class="slbl">🟢 Grün-Erkennungsmaske</div>
   <img src="/stream/mask" alt="Masken-Stream">
  </div>
 </div>
</div>

<script>
const SC={FOLLOWING:'#3fb950',ALIGNING:'#d29922',SEARCHING:'#e3b341',PAUSED:'#f85149'};

function cls(v,w,b){return Math.abs(v)>=b?'bad':Math.abs(v)>=w?'warn':'good'}
function set(id,txt,klass='val'){
  const e=document.getElementById(id);
  if(e){e.textContent=txt;e.className='val '+(klass||'')}
}

function drawArc(deg,limit){
  const c=document.getElementById('arc-canvas');
  const ctx=c.getContext('2d');
  ctx.clearRect(0,0,100,60);
  const cx=50,cy=55,r=42;
  // Hintergrund-Halbbogen
  ctx.beginPath();ctx.arc(cx,cy,r,Math.PI,0);
  ctx.strokeStyle='#21262d';ctx.lineWidth=8;ctx.stroke();
  // Limit-Markierung
  const limAngle=Math.PI - (limit/180)*Math.PI;
  ctx.beginPath();ctx.arc(cx,cy,r,Math.PI,limAngle);
  ctx.strokeStyle='#f85149';ctx.lineWidth=8;ctx.stroke();
  // Aktueller Wert
  const absD=Math.min(Math.abs(deg||0),180);
  const curAngle=Math.PI - (absD/180)*Math.PI;
  ctx.beginPath();ctx.arc(cx,cy,r,Math.PI,curAngle);
  ctx.strokeStyle=absD>=(limit||82)?'#f85149':absD>=(limit||82)*0.7?'#d29922':'#3fb950';
  ctx.lineWidth=8;ctx.stroke();
  // Zeiger
  const a=Math.PI - (absD/180)*Math.PI;
  ctx.beginPath();
  ctx.moveTo(cx,cy);
  ctx.lineTo(cx+r*Math.cos(a),cy-r*Math.sin(a));  // Note: canvas y is inverted
  ctx.strokeStyle='#c9d1d9';ctx.lineWidth=2;ctx.stroke();
  // Label
  ctx.fillStyle='#8b949e';ctx.font='10px Courier New';ctx.textAlign='center';
  ctx.fillText(absD.toFixed(0)+'°',cx,cy-12);
}

const t0=Date.now();

async function poll(){
  try{
    const r=await fetch('/status');
    const d=await r.json();

    const st=d.state||'–';
    const badge=document.getElementById('badge');
    badge.textContent=st;
    badge.style.background=SC[st]||'#21262d';
    badge.style.color=SC[st]?'#000':'#c9d1d9';
    document.getElementById('fps-hdr').textContent='FPS: '+(d.fps??'–');

    set('v-state', st);
    set('v-speed', d.speed!=null ? d.speed.toFixed(2) : '–');
    set('v-espeed',d.eff_speed!=null ? d.eff_speed.toFixed(2) : '–');

    const found=d.path_found;
    const fEl=document.getElementById('v-found');
    fEl.textContent=found?'JA ✓':'NEIN ✗';
    fEl.className='val '+(found?'good':'bad');

    const off=d.offset??0;
    document.getElementById('v-offset').textContent=
      d.offset!=null?(off*100).toFixed(1)+'%':'–';
    document.getElementById('v-offset').className='val '+cls(off,.3,.6);

    // Offset-Balken
    const bar=document.getElementById('bar-fill');
    const pct=(off+1)/2*100;
    const fw=Math.abs(off)*50;
    bar.style.left=(off<0?pct:50)+'%';
    bar.style.width=fw+'%';
    bar.style.background=Math.abs(off)>.6?'#f85149':Math.abs(off)>.3?'#d29922':'#3fb950';

    set('v-area',   d.area!=null ? Math.round(d.area).toLocaleString()+' px²' : '–');
    const dzEl=document.getElementById('v-dz');
    dzEl.textContent=d.in_dead_zone?'JA':'NEIN';
    dzEl.className='val '+(d.in_dead_zone?'good':'');

    const bend=d.bend_angle??0;
    document.getElementById('v-bend').textContent=d.bend_angle!=null?bend.toFixed(1)+'°':'–';
    document.getElementById('v-bend').className='val '+cls(bend,15,32);
    set('v-bdir',  d.bend_dir||'–');
    set('v-sf',    d.speed_factor!=null ? 'x'+d.speed_factor.toFixed(2) : '–');
    const shEl=document.getElementById('v-sharp');
    shEl.textContent=d.is_sharp_bend?'JA ⚠':'NEIN';
    shEl.className='val '+(d.is_sharp_bend?'bad':'good');

    const hdeg=d.heading_deg??0;
    const hlim=d.heading_limit??82;
    document.getElementById('v-hdeg').textContent=hdeg.toFixed(0)+'°';
    document.getElementById('v-hdeg').className='val '+cls(hdeg,hlim*0.7,hlim);
    document.getElementById('v-hlim').textContent=hlim+'°';
    drawArc(hdeg,hlim);

    set('v-fps',    d.fps??'–');
    set('v-frames', d.frame_count??'–');
    const up=Math.round((Date.now()-t0)/1000);
    set('v-uptime', up+'s');

  }catch(e){}
}
setInterval(poll,300);
poll();
</script>
</body>
</html>
"""


# ── Thread-sicherer Frame-Puffer ──────────────────────────────────────────────

class _FrameBuffer:
    """Hält immer den neuesten Frame – ältere werden überschrieben."""

    def __init__(self):
        self._frame: Optional[np.ndarray] = None
        self._lock  = threading.Lock()
        self._event = threading.Event()

    def push(self, frame: np.ndarray):
        with self._lock:
            self._frame = frame
        self._event.set()

    def get(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """Wartet auf neuen Frame (blockierend bis timeout)."""
        self._event.wait(timeout=timeout)
        self._event.clear()
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


# ── Status-Puffer ─────────────────────────────────────────────────────────────

class _StatusBuffer:
    def __init__(self):
        self._data: Dict[str, Any] = {}
        self._lock = threading.Lock()

    def push(self, data: Dict[str, Any]):
        with self._lock:
            self._data = dict(data)

    def get(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._data)


# ── Debug-Server ──────────────────────────────────────────────────────────────

class DebugServer:
    """
    Startet einen Flask-Webserver im Hintergrund-Thread.

    Verwendung in main.py:
        srv = DebugServer(port=5000, stream_fps=15)
        srv.start()
        # Im Loop:
        srv.push(main_frame, mask_frame, status_dict)
    """

    def __init__(self, port: int = 5000, stream_fps: int = 15):
        if not _FLASK_OK:
            raise ImportError(
                "Flask nicht installiert. "
                "Installieren mit:  pip install flask"
            )
        self._port       = port
        self._min_dt     = 1.0 / max(stream_fps, 1)
        self._buf_main   = _FrameBuffer()
        self._buf_mask   = _FrameBuffer()
        self._status     = _StatusBuffer()
        self._app        = self._build_app()
        self._thread: Optional[threading.Thread] = None

    # ── Öffentliche API ───────────────────────────────────────────────────────

    def start(self):
        """Startet den Server in einem Daemon-Thread (stoppt mit dem Hauptprozess)."""
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._run, name="DebugServer", daemon=True
        )
        self._thread.start()
        logger.info(
            "🌐 Debug-Dashboard erreichbar unter  http://<rover-ip>:%d", self._port
        )

    def push(self,
             main_frame: np.ndarray,
             mask_frame: np.ndarray,
             status: Dict[str, Any]):
        """
        Frames und Zustand aktualisieren.
        Wird jeden Kamera-Frame aus dem Hauptloop aufgerufen.

        Args:
            main_frame: BGR-Frame mit Overlays (von PathDetector)
            mask_frame: Grau-Maske (von detector.get_mask_only())
            status:     Dict mit aktuellen Zustandswerten
        """
        self._buf_main.push(main_frame)

        # Maske: in BGR konvertieren damit der Browser sie anzeigen kann
        if len(mask_frame.shape) == 2:
            mask_bgr = cv2.cvtColor(mask_frame, cv2.COLOR_GRAY2BGR)
        else:
            mask_bgr = mask_frame
        # Grüne Pixel hervorheben
        colored = np.zeros_like(mask_bgr)
        colored[mask_frame > 0] = (0, 220, 80)
        self._buf_mask.push(colored)

        self._status.push(status)

    # ── Flask-App aufbauen ────────────────────────────────────────────────────

    def _build_app(self) -> Flask:
        app = Flask(__name__)
        # Logging von Flask/Werkzeug unterdrücken
        import logging as _lg
        _lg.getLogger("werkzeug").setLevel(_lg.ERROR)

        @app.route("/")
        def dashboard():
            return render_template_string(_HTML)

        @app.route("/stream/main")
        def stream_main():
            return Response(
                self._mjpeg_generator(self._buf_main),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @app.route("/stream/mask")
        def stream_mask():
            return Response(
                self._mjpeg_generator(self._buf_mask),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @app.route("/status")
        def status():
            return jsonify(self._status.get())

        return app

    # ── MJPEG-Generator ───────────────────────────────────────────────────────

    def _mjpeg_generator(self, buf: _FrameBuffer):
        """Generator der JPEG-Frames als MJPEG-Stream liefert."""
        last_t = 0.0
        placeholder = self._make_placeholder()

        while True:
            now = time.time()
            # FPS-Limit einhalten
            if now - last_t < self._min_dt:
                time.sleep(0.005)
                continue
            last_t = now

            frame = buf.get(timeout=2.0)
            if frame is None:
                frame = placeholder

            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
            if not ok:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + jpeg.tobytes()
                + b"\r\n"
            )

    @staticmethod
    def _make_placeholder() -> np.ndarray:
        """Platzhalterbild wenn noch kein Frame vorhanden."""
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[:] = (30, 30, 30)
        cv2.putText(img, "Warte auf Kamera...", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
        return img

    # ── Server-Thread ─────────────────────────────────────────────────────────

    def _run(self):
        self._app.run(
            host="0.0.0.0",
            port=self._port,
            threaded=True,
            use_reloader=False,
            debug=False,
        )
