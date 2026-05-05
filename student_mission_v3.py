

import rclpy, threading, time, json, os, subprocess
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from dexi_interfaces.srv import LEDRingColor, ServoControl
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2, numpy as np

# ── HSV color ranges (calibrated for Arducam IMX708) ─────────────────────────
COLORS = {
    "red":    {"lower": [  0,120, 70], "upper": [ 10,255,255],
               "lower2":[170,120, 70], "upper2":[179,255,255],
               "bgr": (0, 0, 255)},
    "blue":   {"lower": [100,100, 50], "upper": [130,255,255],
               "bgr": (255, 0, 0)},
    "white":  {"lower": [  0,  0,200], "upper": [179, 30,255],
               "bgr": (255, 255, 255)},
    "purple": {"lower": [135, 80, 80], "upper": [165,255,255],
               "bgr": (255, 0, 255)},
}

# ── Servo (FS90R continuous rotation on PCA9685 channel 0) ───────────────────
CLAW_CHANNEL   = 0
STOP_ANGLE     = 82   # neutral — stops the servo
OPEN_ANGLE     = 150  # spins open direction
CLOSED_ANGLE   = 25   # spins close direction
MOVE_DURATION  = 1.2  # seconds to run before stopping

# ── Shared state ──────────────────────────────────────────────────────────────
program_running = False
detected_color  = "none"
latest_frame    = b""
frame_lock      = threading.Lock()
color_enabled   = {name: True for name in COLORS}
node_ref        = None
claw_busy       = False
detected_boxes  = []
red_seen        = False
last_red_time   = 0.0
RED_HOLD_TIME   = 0.5  # seconds red must be absent before closing

# ── v3 (Path B) flags ────────────────────────────────────────────────────────
PORT         = 8002
LED_ENABLED  = True
CLAW_ENABLED = True

# ── Lock white balance to daylight (V4L2 control on /dev/video0) ─────────────
# Camera node uses cv::VideoCapture / V4L2 (no libcamera presets), so this is
# auto-off + a Kelvin temperature. Stops AWB grey-world from green-tinting the
# scene when red dominates the frame. dexi user is in `video` group → no sudo.
def apply_daylight_awb():
    try:
        subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "white_balance_automatic=0"],
                       check=False, timeout=2, capture_output=True)
        subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "white_balance_temperature=5500"],
                       check=False, timeout=2, capture_output=True)
    except Exception:
        pass


# ── Embedded web page ─────────────────────────────────────────────────────────
HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DEXI Student Mission</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0d0d1a;font-family:Consolas,monospace;color:#d4d4d4;display:flex;flex-direction:column;height:100vh}
  header{background:#13111f;border-bottom:2px solid #7e22ce;padding:12px 20px;display:flex;align-items:center;gap:12px}
  header h1{color:#c084fc;font-size:1.1rem}
  header span{color:#6b7280;font-size:.8rem;margin-left:4px}
  .layout{display:flex;flex:1;overflow:hidden}
  .feed{flex:1;background:#000;display:flex;flex-direction:column;align-items:center;justify-content:center;position:relative}
  .feed img{width:100%;height:100%;object-fit:contain;transition:transform .2s}
  .feed canvas{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none}
  .rotate-btns{position:absolute;bottom:8px;right:8px;display:flex;gap:4px;z-index:10}
  .rotate-btns button{background:#1e1b2e;border:1px solid #4b5563;color:#d4d4d4;padding:4px 8px;border-radius:4px;cursor:pointer;font-size:.75rem}
  .rotate-btns button:hover{background:#2d2b3e}
  .panel{width:270px;background:#13111f;border-left:2px solid #2e2e3a;display:flex;flex-direction:column;padding:14px;gap:12px;overflow-y:auto}
  .label{font-size:.68rem;font-weight:700;text-transform:uppercase;letter-spacing:.1em;color:#6b7280;margin-bottom:6px}
  .status-box{background:#1e1533;border:1px solid #3b0764;border-radius:8px;padding:10px 12px;display:flex;align-items:center;gap:10px}
  .dot{width:10px;height:10px;border-radius:50%;background:#6b7280;flex-shrink:0}
  .dot.active{animation:pulse .8s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
  .status-text{font-size:.85rem;font-weight:700;color:#d4d4d4}
  .btn{display:block;width:100%;padding:9px;font-family:Consolas,monospace;font-weight:700;font-size:.85rem;border:none;border-radius:7px;cursor:pointer;transition:opacity .15s;margin-bottom:6px}
  .btn:last-child{margin-bottom:0}
  .btn-start{background:#4ade80;color:#052e16}
  .btn-start:hover{opacity:.85}
  .btn-stop{background:#f87171;color:#2a1215}
  .btn-stop:hover{opacity:.85}
  .btn:disabled{background:#2e2e3a!important;color:#6b7280;cursor:not-allowed}
  .toggle-row{display:flex;gap:5px}
  .btn-tog{flex:1;padding:7px 2px;font-size:.72rem;font-weight:700;border:none;border-radius:5px;cursor:pointer;text-transform:uppercase;letter-spacing:.04em;transition:opacity .15s}
  .btn-tog:hover{opacity:.8}
  .btn-tog.off{background:#2e2e3a!important;color:#6b7280!important}
  #tog-red{background:#ef4444;color:#fff}
  #tog-blue{background:#3b82f6;color:#fff}
  #tog-white{background:#ffffff;color:#000;border:1px solid #ccc}
  #tog-purple{background:#8b5cf6;color:#fff}
  .divider{border:none;border-top:1px solid #2e2e3a;margin:2px 0}
  .storage-bar-bg{background:#1e1533;border-radius:4px;height:8px;overflow:hidden;margin:6px 0 4px}
  .storage-bar-fill{height:100%;border-radius:4px;background:#a855f7;transition:width .6s}
  .storage-text{font-size:.72rem;color:#9ca3af}
  .sysinfo-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin-top:4px}
  .sysinfo-val{background:#1e1533;border-radius:6px;padding:8px 4px;text-align:center}
  .sysinfo-val .val{font-size:1.1rem;font-weight:700}
  .sysinfo-val .lbl{font-size:.65rem;color:#6b7280;margin-top:2px}
  .servo-row{display:flex;align-items:center;gap:8px;margin-bottom:8px}
  .servo-row input[type=range]{flex:1;accent-color:#3b82f6}
  .servo-val{font-size:.85rem;font-weight:700;min-width:38px;text-align:right;color:#3b82f6}
  .btn-servo{background:#3b82f6;color:#fff}
  .btn-servo:hover{opacity:.85}
  .btn-servo-stop{background:#f87171;color:#2a1215}
  .btn-servo-stop:hover{opacity:.85}
</style>
</head>
<body>

<header>
  <h1>DEXI <span>Student Mission</span></h1>
</header>

<div class="layout">
  <div class="feed">
    <img id="feed" src="/stream">
    <canvas id="overlay" width="320" height="240"></canvas>
    <div class="rotate-btns">
      <button onclick="rotCam(-90)">&#8634; 90&deg;</button>
      <button onclick="rotCam(90)">&#8635; 90&deg;</button>
      <button onclick="rotCam(180)">&#8597; 180&deg;</button>
    </div>
  </div>
  <div class="panel">

    <!-- Program status -->
    <div>
      <div class="label">Program</div>
      <div class="status-box">
        <div class="dot" id="prog-dot"></div>
        <div class="status-text" id="prog-text">Stopped</div>
      </div>
    </div>

    <!-- Detected color -->
    <div>
      <div class="label">Detected Color</div>
      <div class="status-box">
        <div class="dot" id="det-dot"></div>
        <div class="status-text" id="det-text">None</div>
      </div>
    </div>

    <!-- Start / Stop -->
    <div>
      <button class="btn btn-start" id="start-btn" onclick="startMission()">&#9654; Start Mission</button>
      <button class="btn btn-stop"  id="stop-btn"  onclick="stopMission()">&#9632; Stop Mission</button>
    </div>

    <hr class="divider">

    <!-- Color toggles -->
    <div>
      <div class="label">Color Detection</div>
      <div class="toggle-row">
        <button class="btn-tog" id="tog-red"    onclick="toggle('red')">Red</button>
        <button class="btn-tog" id="tog-blue"   onclick="toggle('blue')">Blue</button>
        <button class="btn-tog" id="tog-white"  onclick="toggle('white')">White</button>
        <button class="btn-tog" id="tog-purple" onclick="toggle('purple')">Purple</button>
      </div>
    </div>

    <hr class="divider">

    <!-- Servo / Claw -->
    <div>
      <div class="label">Servo / Claw</div>
      <div class="servo-row">
        <input type="range" id="servo-slider" min="0" max="180" value="82"
               oninput="sendServo()">
        <span class="servo-val" id="servo-val">82</span>
      </div>
      <button class="btn btn-servo"      onclick="document.getElementById('servo-slider').value=25; sendServo(25);">Open Claw</button>
      <button class="btn btn-servo"      onclick="document.getElementById('servo-slider').value=180; sendServo(180);">Close Claw</button>
    </div>

    <hr class="divider">

    <!-- Storage -->
    <div>
      <div class="label">Storage</div>
      <div class="storage-bar-bg"><div class="storage-bar-fill" id="stor-fill" style="width:0%"></div></div>
      <div class="storage-text" id="stor-text">—</div>
    </div>

    <!-- System info -->
    <div>
      <div class="label">System</div>
      <div class="sysinfo-grid">
        <div class="sysinfo-val"><div class="val" id="si-ma">—</div><div class="lbl">mA</div></div>
        <div class="sysinfo-val"><div class="val" id="si-temp">—</div><div class="lbl">°C</div></div>
        <div class="sysinfo-val"><div class="val" id="si-cpu">—</div><div class="lbl">CPU%</div></div>
      </div>
    </div>

  </div>
</div>

<script>
let _rot = 0;
function rotCam(deg){
  _rot = (_rot + deg + 360) % 360;
  document.getElementById('feed').style.transform = 'rotate(' + _rot + 'deg)';
}

function toggle(color){
  fetch('/api/toggle',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({color})});
}

function startMission(){ fetch('/api/start',{method:'POST'}); }
function stopMission(){  fetch('/api/stop', {method:'POST'}); }

let _servoBusy=false, _servoPending=null;
async function _flushServo(){
  while(_servoPending!==null){
    const a=_servoPending; _servoPending=null;
    await fetch('/api/servo',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({angle:a})});
  }
  _servoBusy=false;
}
function sendServo(angle){
  const a = angle !== undefined ? angle : parseInt(document.getElementById('servo-slider').value);
  document.getElementById('servo-val').textContent = a;
  _servoPending = a;
  if(!_servoBusy){ _servoBusy=true; _flushServo(); }
}

async function poll(){
  try{
    const d = await (await fetch('/api/status')).json();
    const run = d.running;
    document.getElementById('prog-dot').className   = 'dot'+(run?' active':'');
    document.getElementById('prog-dot').style.background = run?'#4ade80':'#6b7280';
    document.getElementById('prog-text').textContent = run?'Running':'Stopped';
    const _detCss = {red:'#ef4444', blue:'#3b82f6', white:'#ffffff', purple:'#8b5cf6'};
    const _dot = document.getElementById('det-dot');
    const _txt = document.getElementById('det-text');
    if(d.color && d.color!=='none' && _detCss[d.color]){
      _dot.style.background = _detCss[d.color];
      _txt.style.color      = _detCss[d.color];
      _txt.textContent      = d.color;
    }else{
      _dot.style.background = '#6b7280';
      _txt.style.color      = '#d4d4d4';
      _txt.textContent      = 'none';
    }
    document.getElementById('start-btn').disabled = run;
    document.getElementById('stop-btn').disabled  = !run;
    const ce = d.color_enabled||{};
    for(const c of ['red','blue','white','purple']){
      const b=document.getElementById('tog-'+c);
      if(b) b.classList.toggle('off',!ce[c]);
    }
  }catch(e){}
  setTimeout(poll,800);
}

async function pollStorage(){
  try{
    const d=await (await fetch('/api/storage')).json();
    if(d.percent!==undefined){
      document.getElementById('stor-fill').style.width=d.percent+'%';
      document.getElementById('stor-text').textContent=`${d.used} / ${d.total} (${d.percent}%)`;
    }
  }catch(e){}
  setTimeout(pollStorage,15000);
}

async function pollSysinfo(){
  try{
    const d=await (await fetch('/api/sysinfo')).json();
    const ma=Math.round(d.mA||0),temp=Math.round(d.temp_c||0),cpu=Math.round(d.cpu_pct||0);
    const elMA=document.getElementById('si-ma'),elTemp=document.getElementById('si-temp'),elCPU=document.getElementById('si-cpu');
    elMA.textContent=ma; elTemp.textContent=temp; elCPU.textContent=cpu;
    elMA.style.color  = ma  <700?'#4ade80':ma  <1100?'#facc15':ma  <1400?'#fb923c':'#f87171';
    elTemp.style.color= temp< 55?'#4ade80':temp<  70?'#facc15':'#f87171';
    elCPU.style.color = cpu < 40?'#4ade80':cpu<  70?'#facc15':'#f87171';
  }catch(e){}
  setTimeout(pollSysinfo,2000);
}

poll(); pollStorage(); pollSysinfo();

// ── Detection bounding-box overlay (DISABLED — uncomment last 2 lines for debug) ──
// Polls /api/detection at 2 Hz and draws a box around the priority detection on
// the canvas overlay. Disabled because we only want the colour indicator
// (det-dot/det-text), not on-screen bounding boxes.
const _ovl=document.getElementById('overlay');
const _ctx=_ovl.getContext('2d');
async function pollDetection(){
  try{
    const d=await (await fetch('/api/detection')).json();
    _ctx.clearRect(0,0,_ovl.width,_ovl.height);
    _ctx.lineWidth=2;
    _ctx.font='bold 14px Consolas,monospace';
    for(const b of (d.boxes||[])){
      const c=b.css||'#fff';
      _ctx.strokeStyle=c; _ctx.fillStyle=c;
      _ctx.strokeRect(b.x,b.y,b.w,b.h);
      _ctx.fillText(b.color||'',b.x,Math.max(12,b.y-4));
    }
  }catch(e){}
}
// setInterval(pollDetection,500);
// pollDetection();
</script>
</body>
</html>"""

# ── HTTP handler ──────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def do_GET(self):
        if self.path == "/":
            body = HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with frame_lock: f = latest_frame
                    if f:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n")
                    time.sleep(0.04)
            except: pass

        elif self.path == "/api/status":
            body = json.dumps({
                "running":         program_running,
                "color":           detected_color,
                "color_enabled":   color_enabled,
                "claw_busy":       claw_busy,
            }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/api/sysinfo":
            try:
                cpu = float(open('/proc/loadavg').read().split()[0]) * 100 / os.cpu_count()
                cpu = min(cpu, 100.0)
                temp_c = int(open('/sys/class/thermal/thermal_zone0/temp').read().strip()) / 1000.0
                mA = 500 + (cpu / 100.0) * 1000
                body = json.dumps({"cpu_pct": cpu, "temp_c": temp_c, "mA": mA}).encode()
            except Exception as e:
                body = json.dumps({"cpu_pct": 0, "temp_c": 0, "mA": 500, "error": str(e)}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/api/storage":
            try:
                import subprocess as _sp
                out = _sp.check_output(["df", "-h", "/"], text=True).strip().split("\n")[1].split()
                body = json.dumps({
                    "total": out[1], "used": out[2],
                    "avail": out[3], "percent": int(out[4].replace("%", ""))
                }).encode()
            except Exception as e:
                body = json.dumps({"error": str(e)}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/api/detection":
            body = json.dumps({
                "boxes":     detected_boxes,
                "color":     detected_color,
                "running":   program_running,
                "claw_busy": claw_busy,
            }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        global program_running, color_enabled

        length = int(self.headers.get("Content-Length", 0))
        body   = json.loads(self.rfile.read(length)) if length else {}
        resp   = b'{"ok":true}'

        if self.path == "/api/start":
            program_running = True

        elif self.path == "/api/stop":
            program_running = False

        elif self.path == "/api/toggle":
            color = body.get("color", "")
            if color in color_enabled:
                color_enabled[color] = not color_enabled[color]

        elif self.path == "/api/servo":
            angle = int(body.get("angle", STOP_ANGLE))
            angle = max(0, min(180, angle))
            if node_ref:
                node_ref.send_servo(angle)


        elif self.path == "/api/open_claw":
            if node_ref:
                threading.Thread(
                    target=node_ref.manual_open_claw, daemon=True).start()
            import pathlib
            pathlib.Path("/tmp/claw_open_signal").touch()
            resp = json.dumps({"ok": True}).encode()

        elif self.path == "/api/close_claw":
            if node_ref:
                threading.Thread(
                    target=node_ref.manual_close_claw, daemon=True).start()
            resp = json.dumps({"ok": True}).encode()

        elif self.path == "/api/reboot":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
            threading.Thread(target=lambda: (time.sleep(1), subprocess.run(["sudo", "reboot"])), daemon=True).start()
            return

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(resp)


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class StudentMission(Node):
    def __init__(self):
        global node_ref
        super().__init__("student_mission")
        self._led_color = None
        node_ref = self

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.led = self.create_client(LEDRingColor, "/dexi/led_service/set_led_ring_color")
        self.led.wait_for_service(timeout_sec=5.0)

        self.servo = self.create_client(ServoControl, "/dexi/servo_control")
        self.servo.wait_for_service(timeout_sec=5.0)

        # v3 (Path B): two subscriptions
        #   - full-rate topic feeds the MJPEG stream untouched (zero CPU image work)
        #   - throttled 2 Hz topic feeds the detection pass
        self.create_subscription(CompressedImage, "/cam0/image_raw/compressed",     self.cb_stream, qos)
        self.create_subscription(CompressedImage, "/cam0/image_raw/compressed_2hz", self.cb_detect, qos)
        if LED_ENABLED:
            self.set_led("white")
        if CLAW_ENABLED:
            self.send_servo(OPEN_ANGLE)
        self.get_logger().info(f"Student Mission v3 ready — open http://192.168.4.1:{PORT}")

    def set_led(self, color, force=False):
        if self._led_color == color and not force:
            return
        req = LEDRingColor.Request()
        req.color = color
        self.led.call_async(req)
        self._led_color = color

    def manual_open_claw(self):
        self.send_servo(OPEN_ANGLE)

    def manual_close_claw(self):
        self.send_servo(CLOSED_ANGLE)

    def send_servo(self, angle: int):
        req = ServoControl.Request()
        req.pin    = CLAW_CHANNEL
        req.angle  = angle
        req.min_pw = 0
        req.max_pw = 0
        self.servo.call_async(req)

    def _claw_open(self):
        global claw_busy
        try:
            self.send_servo(OPEN_ANGLE)
            time.sleep(1.5)
            self.send_servo(STOP_ANGLE)
        finally:
            claw_busy = False

    def _claw_close(self):
        global claw_busy
        try:
            self.send_servo(CLOSED_ANGLE)
            time.sleep(1.5)
            self.send_servo(STOP_ANGLE)
        finally:
            claw_busy = False


#-----------------------------------------------------------------------------------------------
    # ── v3 (Path B) callbacks ────────────────────────────────────────────────
    # Stream callback: fires at the camera's full rate (~30 Hz). Pure pass-through —
    # the JPEG bytes from the camera node go straight to the MJPEG response.
    def cb_stream(self, msg):
        global latest_frame
        with frame_lock:
            latest_frame = bytes(msg.data)

    # Detection callback: fires at 2 Hz (throttled topic). Decodes once, runs HSV/
    # contours, and stores results in `detected_boxes` for the browser to overlay.
    def cb_detect(self, msg):
        global detected_color, detected_boxes, claw_busy, red_seen, last_red_time

        if not program_running:
            detected_color = "none"
            detected_boxes = []
            if LED_ENABLED:
                self.set_led("white")
            return

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Priority by area: pick the colour whose largest contour has the biggest
        # area above threshold, not the first one in COLORS dict order. Only the
        # winning colour ends up in `boxes` (debug overlay shows just that one).
        best_name, best_area, best_contour, best_cfg = None, 0.0, None, None
        for name, cfg in COLORS.items():
            if not color_enabled.get(name, True):
                continue
            mask = cv2.inRange(hsv, np.array(cfg["lower"]), np.array(cfg["upper"]))
            if "lower2" in cfg:
                mask = cv2.bitwise_or(mask,
                    cv2.inRange(hsv, np.array(cfg["lower2"]), np.array(cfg["upper2"])))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            biggest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest)
            if area > 1500 and area > best_area:
                best_name, best_area, best_contour, best_cfg = name, area, biggest, cfg

        boxes = []
        found = best_name
        if best_name is not None:
            x, y, w, h = cv2.boundingRect(best_contour)
            b, g, r    = best_cfg["bgr"]
            boxes.append({
                "color": best_name,
                "x": int(x), "y": int(y), "w": int(w), "h": int(h),
                "css":   f"rgb({r},{g},{b})",
            })

        if CLAW_ENABLED:
            # White is detectable and can win the priority pick (UI/LED reflect it),
            # but the claw state machine treats it as a no-op so a white surface
            # winning over a fading red blob doesn't cause a false close. White
            # being disabled in the toggles makes this moot — culled before pick.
            if found == "white":
                pass
            elif found == "red":
                last_red_time = time.time()
                if not red_seen and not claw_busy:
                    red_seen  = True
                    claw_busy = True
                    threading.Thread(target=self._claw_open, daemon=True).start()
            else:
                if not found:
                    red_seen = False
                if red_seen and not claw_busy and (time.time() - last_red_time > RED_HOLD_TIME):
                    red_seen  = False
                    claw_busy = True
                    threading.Thread(target=self._claw_close, daemon=True).start()

        detected_color = found or "none"
        detected_boxes = boxes
        if LED_ENABLED:
            self.set_led(found if found else "white")


def main():
    server = ThreadingHTTPServer(("", PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Student Mission v3 web page: http://192.168.4.1:{PORT}")
    apply_daylight_awb()

    rclpy.init()
    rclpy.spin(StudentMission())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
