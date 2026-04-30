

import rclpy, threading, time, json, os, subprocess, signal
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from dexi_interfaces.srv import LEDRingColor, ServoControl
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2, numpy as np

# SKIP to line 570

def html_base():
    # ── Embedded web page ─────────────────────────────────────────────────────────
    html = """


<!DOCTYPE html>
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
  #tog-orange{background:#f97316;color:#fff}
  #tog-green{background:#22c55e;color:#052e16}
  #tog-blue{background:#3b82f6;color:#fff}
  #tog-white {background: #ffffff;color: #000000;border: 1px solid #ccc;}
  #tog-purple {background: #8b5cf6;color: #fff;}


  .divider{border:none;border-top:1px solid #2e2e3a;margin:2px 0}
  .btn-upload{background:#7e22ce;color:#e9d5ff}
  .btn-upload:hover{opacity:.85}
  .mission-select{width:100%;background:#1e1533;color:#d4d4d4;border:1px solid #3b0764;border-radius:5px;padding:7px 8px;font-family:Consolas,monospace;font-size:.8rem;margin-bottom:7px;cursor:pointer}
  .mission-select option{background:#1e1533}
  .run-row{display:flex;gap:6px}
  .run-row .btn{margin-bottom:0}
  .btn-run{background:#4ade80;color:#052e16}
  .btn-run-stop{background:#f87171;color:#2a1215}
  .mission-status{font-size:.73rem;color:#6b7280;margin-top:5px;word-break:break-all;min-height:16px}
  .toggle-row-setting{display:flex;align-items:center;gap:10px;margin-bottom:6px}
  .toggle-sw{position:relative;display:inline-block;width:44px;height:24px;flex-shrink:0}
  .toggle-sw input{opacity:0;width:0;height:0}
  .toggle-sl{position:absolute;cursor:pointer;inset:0;background:#2e2e3a;border-radius:24px;transition:.3s}
  .toggle-sl:before{position:absolute;content:"";height:18px;width:18px;left:3px;bottom:3px;background:#d4d4d4;border-radius:50%;transition:.3s}
  input:checked+.toggle-sl{background:#7e22ce}
  input:checked+.toggle-sl:before{transform:translateX(20px)}
  .toggle-label{font-size:.8rem;font-weight:700;color:#d4d4d4}
  .setting-hint{font-size:.7rem;color:#6b7280;margin-top:3px}
  .btn-reboot{background:#dc2626;color:#fff}
  .btn-reboot:hover{opacity:.85}
  .btn-save{background:#2563eb;color:#eff6ff}
  .btn-save:hover{opacity:.85}
  .btn-clear{background:#2e2e3a;color:#d4d4d4}
  .btn-clear:hover{opacity:.85}
  .startup-status{font-size:.73rem;margin-top:5px;min-height:16px}
  .storage-bar-bg{background:#1e1533;border-radius:4px;height:8px;overflow:hidden;margin:6px 0 4px}
  .storage-bar-fill{height:100%;border-radius:4px;background:#a855f7;transition:width .6s}
  .storage-text{font-size:.72rem;color:#9ca3af}
  .sysinfo-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin-top:4px}
  .sysinfo-val{background:#1e1533;border-radius:6px;padding:8px 4px;text-align:center}
  .sysinfo-val .val{font-size:1.1rem;font-weight:700}
  .sysinfo-val .lbl{font-size:.65rem;color:#6b7280;margin-top:2px}
  .overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.75);z-index:100;align-items:center;justify-content:center}
  .overlay.show{display:flex}
  .modal{background:#13111f;border:2px solid #7e22ce;border-radius:10px;padding:24px;width:310px}
  .modal h3{color:#c084fc;margin-bottom:16px;font-size:1rem}
  .modal label{font-size:.72rem;color:#6b7280;display:block;margin-bottom:4px;text-transform:uppercase;letter-spacing:.08em}
  .modal input[type=text]{width:100%;background:#1e1533;border:1px solid #3b0764;color:#d4d4d4;padding:8px;border-radius:5px;font-family:Consolas,monospace;font-size:.85rem;margin-bottom:12px}
  .modal input[type=file]{color:#d4d4d4;font-size:.78rem;margin-bottom:14px;display:block}
  .modal-btns{display:flex;gap:8px}
  .modal-btns .btn{margin-bottom:0}
  .btn-cancel{background:#2e2e3a;color:#d4d4d4}
  #upload-msg{font-size:.75rem;margin-top:10px;min-height:16px;color:#a855f7}
  .servo-row{display:flex;align-items:center;gap:8px;margin-bottom:8px}
  .servo-row input[type=range]{flex:1;accent-color:#3b82f6}
  .servo-val{font-size:.85rem;font-weight:700;min-width:38px;text-align:right;color:#3b82f6}
  .claw-status{font-size:.73rem;margin-top:5px;min-height:16px;color:#6b7280}
  .btn-servo{background:#3b82f6;color:#fff}
  .btn-servo:hover{opacity:.85}
  .btn-servo-stop{background:#f87171;color:#2a1215}
  .btn-servo-stop:hover{opacity:.85}
</style>
</head>
<body>

<!-- Upload modal -->
<div class="overlay" id="upload-overlay">
  <div class="modal">
    <h3>&#128196; Upload Your Mission</h3>
    <label>Your Name</label>
    <input type="text" id="student-name" placeholder="e.g. alex">
    <label>Python File (.py)</label>
    <input type="file" id="mission-file" accept=".py">
    <div class="modal-btns">
      <button class="btn btn-upload" onclick="doUpload()">&#8593; Upload</button>
      <button class="btn btn-cancel" onclick="closeUpload()">Cancel</button>
    </div>
    <div id="upload-msg"></div>
  </div>
</div>

<header>
  <h1>DEXI <span>Student Mission</span></h1>
</header>

<div class="layout">
  <div class="feed">
    <img id="feed" src="/stream">
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
        <button class="btn-tog" id="tog-red"    onclick="toggleColor('red')">Red</button>
        <!--<button class="btn-tog" id="tog-orange" onclick="toggleColor('orange')">Orng</button>
        <button class="btn-tog" id="tog-green"  onclick="toggleColor('green')">Green</button>-->
        <button class="btn-tog" id="tog-blue"   onclick="toggleColor('blue')">Blue</button>
        <button class="btn-tog" id="tog-purple"   onclick="toggleColor('purple')">Purple</button>
        <button class="btn-tog" id="tog-white"   onclick="toggleColor('white')">White</button>

      </div>
    </div>

    <hr class="divider">

    <!-- Claw Servo -->
    <div>
      <div class="label">Claw Servo (ch 0)</div>
      <div class="run-row" style="margin-bottom:6px">
        <button class="btn btn-servo" onclick="closeClaw()">&#8679; Open Claw</button>
        <button class="btn" style="background:#28a745;color:white;padding:6px 14px;border:none;border-radius:4px;cursor:pointer;margin-left:6px;" onclick="openClaw()">
            &#8681; Close Claw</button>
      </div>
      <div class="claw-status" id="claw-status">Idle</div>
    </div>

    <hr class="divider">

    <!-- Student missions -->
    <div>
      <div class="label">Student Missions</div>
      <button class="btn btn-upload" onclick="openUpload()" style="margin-bottom:8px">&#8593; Upload Mission</button>
      <select class="mission-select" id="mission-select" onclick="refreshMissions()" onchange="updateRunStopBtns()">
        <option value="">-- select a mission --</option>
      </select>
      <div class="run-row">
        <button class="btn btn-run"      id="run-btn"          onclick="runSelected()">&#9654; Run</button>
        <button class="btn btn-run-stop" id="stop-btn-mission" onclick="stopSelected()">&#9632; Stop</button>
      </div>
      <div class="mission-status" id="mission-status">No mission running</div>
    </div>

    <hr class="divider">

    <!-- Auto-start on boot -->
    <div>
      <div class="label">&#128268; Auto-Start on Boot</div>
      <select class="mission-select" id="startup-select">
        <option value="">-- none --</option>
      </select>
      <div class="run-row">
        <button class="btn btn-save"  onclick="saveStartup()">&#128190; Save</button>
        <button class="btn btn-clear" onclick="clearStartup()">&#10005; Clear</button>
      </div>
      <div class="startup-status" id="startup-status">Loading...</div>
    </div>

    <hr class="divider">

    <!-- System stats -->
    <div>
      <div class="label">System Stats</div>
      <div class="sysinfo-grid">
        <div class="sysinfo-val"><div class="val" id="si-ma">--</div><div class="lbl">mA</div></div>
        <div class="sysinfo-val"><div class="val" id="si-temp">--</div><div class="lbl">&deg;C</div></div>
        <div class="sysinfo-val"><div class="val" id="si-cpu">--</div><div class="lbl">CPU %</div></div>
      </div>
    </div>

    <!-- Pi storage -->
    <div>
      <div class="label">Pi Storage</div>
      <div class="storage-bar-bg">
        <div class="storage-bar-fill" id="storage-fill" style="width:0%"></div>
      </div>
      <div class="storage-text" id="storage-text">Loading...</div>
    </div>

    <hr class="divider">

    <!-- Flight mode LED toggle -->
    <div>
      <div class="label">Flight Mode LED</div>
      <div class="toggle-row-setting">
        <label class="toggle-sw">
          <input type="checkbox" id="flight-toggle" onchange="setFlightMode(this.checked)">
          <span class="toggle-sl"></span>
        </label>
        <span class="toggle-label" id="flight-label">Enabled</span>
      </div>
      <div class="setting-hint">Off = student missions control LEDs freely. Saves across reboots.</div>
      <div class="startup-status" id="flight-status"></div>
    </div>

    <hr class="divider">
    <button class="btn btn-reboot" onclick="rebootDexi()">&#9211; Reboot DEXI</button>

  </div>
</div>

<script>
const COLOR_HEX = {
  red: '#ef4444',
  orange: '#f97316',
  green: '#22c55e',
  blue: '#3b82f6',
  purple: '#8b5cf6',
  white: '#ffffff'
};

async function startMission(){
  document.getElementById('start-btn').disabled = true;
  await fetch('/api/start', {method:'POST'});
  document.getElementById('start-btn').disabled = false;
}
async function stopMission(){
  document.getElementById('stop-btn').disabled = true;
  await fetch('/api/stop', {method:'POST'});
  document.getElementById('stop-btn').disabled = false;
}

async function toggleColor(color){
  await fetch('/api/toggle', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({color})});
}

// ── Servo ─────────────────────────────────────────────────────────────────────
async function closeClaw(){
  await fetch('/api/close_claw',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
}
async function openClaw(){
  const r = await fetch('/api/open_claw',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  const d = await r.json();
  console.log('open claw:', d);
}

function openUpload(){
  document.getElementById('upload-overlay').classList.add('show');
  document.getElementById('student-name').value = '';
  document.getElementById('mission-file').value = '';
  document.getElementById('upload-msg').textContent = '';
}
function closeUpload(){
  document.getElementById('upload-overlay').classList.remove('show');
}
async function doUpload(){
  const name = document.getElementById('student-name').value.trim();
  const file = document.getElementById('mission-file').files[0];
  const msg  = document.getElementById('upload-msg');
  if(!name){ msg.textContent = 'Enter your name first.'; return; }
  if(!file){ msg.textContent = 'Select a .py file first.'; return; }
  msg.textContent = 'Uploading...';
  const content = await file.text();
  const r = await fetch('/api/upload', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({student_name: name, content})});
  const d = await r.json();
  if(d.ok){
    msg.textContent = 'Saved as: ' + d.filename;
    refreshMissions();
    setTimeout(closeUpload, 1800);
  } else {
    msg.textContent = 'Error: ' + (d.error || 'unknown');
  }
}

async function refreshMissions(){
  const d = await (await fetch('/api/missions')).json();
  const sel = document.getElementById('mission-select');
  const cur = sel.value;
  sel.innerHTML = d.missions.map(f =>
    '<option value="' + f + '">' + (f === 'student_mission_v2.py' ? '&#11088; ' + f + ' (default)' : f) + '</option>'
  ).join('');
  if(cur && d.missions.includes(cur)) sel.value = cur;
  else if(d.missions.includes('student_mission.py')) sel.value = 'student_mission.py';
  updateRunStopBtns();
}
function updateRunStopBtns(){
  const sel = document.getElementById('mission-select');
  const locked = sel.value === 'student_mission.py' || sel.value === '';
  document.getElementById('run-btn').disabled  = locked;
  document.getElementById('stop-btn-mission').disabled = locked;
}

async function runSelected(){
  const f = document.getElementById('mission-select').value;
  if(!f) return;
  await fetch('/api/run_mission', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({filename: f})});
}
async function stopSelected(){
  await fetch('/api/stop_mission', {method:'POST'});
}

let _camRot = 0;
function rotCam(deg){
  _camRot = (_camRot + deg + 360) % 360;
  document.getElementById('feed').style.transform = `rotate(${_camRot}deg)`;
}

async function poll(){
  try {
    const d = await (await fetch('/api/status')).json();

    const pd = document.getElementById('prog-dot');
    const pt = document.getElementById('prog-text');
    if(d.running){
      pd.style.background='#4ade80'; pd.classList.add('active');
      pt.textContent = 'Running';
    } else {
      pd.style.background='#6b7280'; pd.classList.remove('active');
      pt.textContent = 'Stopped';
    }

    const dd = document.getElementById('det-dot');
    const dt = document.getElementById('det-text');
    if(d.color !== 'none'){
      const hex = COLOR_HEX[d.color] || '#d4d4d4';
      dd.style.background = hex; dd.classList.add('active');
      dt.textContent = d.color.charAt(0).toUpperCase() + d.color.slice(1);
      dt.style.color = hex;
    } else {
      dd.style.background='#6b7280'; dd.classList.remove('active');
      dt.textContent = 'None'; dt.style.color='#d4d4d4';
    }

    for(const [color, enabled] of Object.entries(d.color_enabled)){
      const btn = document.getElementById('tog-' + color);
      if(btn) btn.classList.toggle('off', !enabled);
    }

    const ms = document.getElementById('mission-status');
    if(d.mission_running){
      ms.textContent = 'Running: ' + d.mission_name;
      ms.style.color = '#4ade80';
    } else {
      ms.textContent = 'No mission running';
      ms.style.color = '#6b7280';
    }

    // Claw status
    const cs = document.getElementById('claw-status');
    if(d.claw_busy){
      cs.textContent = 'Claw sequence running...';
      cs.style.color = '#3b82f6';
    } else {
      cs.textContent = 'Idle';
      cs.style.color = '#6b7280';
    }
  } catch(e){}
  setTimeout(poll, 500);
}

async function loadStartup(){
  const [sd, md] = await Promise.all([
    fetch('/api/startup').then(r=>r.json()),
    fetch('/api/missions').then(r=>r.json())
  ]);
  const sel = document.getElementById('startup-select');
  sel.innerHTML = '<option value="">-- none --</option>' +
    md.missions.map(f => '<option value="'+f+'">' + (f==='student_mission.py'?'&#11088; '+f+' (default)':f) + '</option>').join('');
  if(sd.mission) sel.value = sd.mission;
  setStartupStatus(sd.mission);
}
function setStartupStatus(mission){
  const s = document.getElementById('startup-status');
  if(mission){
    s.textContent = '\u2713 Saved: ' + mission;
    s.style.color = '#4ade80';
  } else {
    s.textContent = "Not set \u2014 won't auto-start";
    s.style.color = '#6b7280';
  }
}
async function saveStartup(){
  const f = document.getElementById('startup-select').value;
  const r = await fetch('/api/set_startup', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({filename: f})});
  const d = await r.json();
  setStartupStatus(d.ok && f ? f : null);
  if(!f) document.getElementById('startup-select').value = '';
}
async function clearStartup(){
  await fetch('/api/clear_startup', {method:'POST'});
  document.getElementById('startup-select').value = '';
  setStartupStatus(null);
}

async function pollStorage(){
  try {
    const d = await (await fetch('/api/storage')).json();
    if(d.percent !== undefined){
      const pct = d.percent;
      const fill = document.getElementById('storage-fill');
      const text = document.getElementById('storage-text');
      fill.style.width = pct + '%';
      fill.style.background = pct > 85 ? '#f87171' : pct > 65 ? '#fbbf24' : '#a855f7';
      text.textContent = d.used + ' used of ' + d.total + ' (' + pct + '%)  \u2014  ' + d.avail + ' free';
    }
  } catch(e){}
  setTimeout(pollStorage, 15000);
}

async function loadFlightMode(){
  const d = await (await fetch('/api/flight_mode')).json();
  document.getElementById('flight-toggle').checked = d.enabled;
  document.getElementById('flight-label').textContent = d.enabled ? 'Enabled' : 'Disabled';
}
async function setFlightMode(enabled){
  document.getElementById('flight-label').textContent = enabled ? 'Enabled' : 'Disabled';
  const s = document.getElementById('flight-status');
  s.textContent = 'Saving...'; s.style.color='#a855f7';
  const r = await fetch('/api/flight_mode', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({enabled})});
  const d = await r.json();
  s.textContent = d.ok ? (enabled ? 'Enabled & saved' : 'Disabled & saved') : 'Error: '+d.error;
  s.style.color = d.ok ? '#4ade80' : '#f87171';
}

async function rebootDexi(){
  if(!confirm('Reboot DEXI? Web will be back in ~30 seconds.')) return;
  await fetch('/api/reboot', {method:'POST'});
}

async function pollSysinfo(){
  try {
    const d = await (await fetch('/api/sysinfo')).json();
    const ma = Math.round(d.mA);
    const temp = d.temp_c.toFixed(1);
    const cpu = Math.round(d.cpu_pct);
    const elMA   = document.getElementById('si-ma');
    const elTemp = document.getElementById('si-temp');
    const elCPU  = document.getElementById('si-cpu');
    elMA.textContent   = ma;
    elTemp.textContent = temp;
    elCPU.textContent  = cpu;
    elMA.style.color = ma < 700 ? '#4ade80' : ma < 1100 ? '#facc15' : ma < 1400 ? '#fb923c' : '#f87171';
    elTemp.style.color = temp < 55 ? '#4ade80' : temp < 70 ? '#facc15' : '#f87171';
    elCPU.style.color  = cpu < 40 ? '#4ade80' : cpu < 70 ? '#facc15' : '#f87171';
  } catch(e){}
  setTimeout(pollSysinfo, 2000);
}

poll();
refreshMissions();
pollStorage();
loadStartup();
loadFlightMode();
pollSysinfo();
</script>
</body>
</html>

"""
    return html.strip()


# ── HSV color ranges (calibrated for Arducam IMX708) ─────────────────────────
COLORS = {
    "red":    {"lower": [ 0,120, 70], "upper": [10,255,255],
               "lower2":[170,120, 70], "upper2":[179,255,255],
               "bgr": (0, 0, 255)},
    #"orange": {"lower": [11,150,100], "upper": [25,255,255],"bgr": (0, 140, 255)},
    #"green":  {"lower": [35, 60, 50], "upper": [85,255,255], "bgr": (0, 200,   0)},


    "blue":   {"lower": [95, 80, 50], "upper": [130,255,255],
               "bgr": (255,  0,   0)},
    "white": {
              "lower": [0, 0, 200],
              "upper": [179, 30, 255],
              "bgr": (255, 255, 255)},

    "purple": {
            "lower": [125, 50, 50],
            "upper": [155, 255, 255],
            "bgr": (255, 0, 255)
}
}

HTML = html_base()

# ── Servo (FS90R continuous rotation on PCA9685 channel 0) ───────────────────

# MAX CLOSE IS 150, MAX OPEN IS 0
CLAW_LOCK = threading.Lock()
CURRENT_ANGLE = 0
CLAW_CHANNEL   = 0
STOP_ANGLE     = 82   # neutral — stops the servo
OPEN_ANGLE     = 150 # spins open direction
CLOSED_ANGLE   = 32   # spins close direction
MOVE_DURATION  = 0.5  # seconds to run before stopping

# ── Shared state ──────────────────────────────────────────────────────────────
program_running     = False
detected_color      = "none"
latest_frame        = b""
frame_lock          = threading.Lock()
color_enabled       = {name: True for name in COLORS}
active_mission_proc = None
active_mission_name = ""
mission_lock        = threading.Lock()
node_ref            = None
claw_busy           = False
servo_angle         = STOP_ANGLE
BUILTIN             = {"my_first_mission.py"}
MISSIONS_DIR        = "/home/dexi/dexi_ws/src"
DEFAULT_MISSION     = "student_mission.py"
STARTUP_FILE        = "/home/dexi/.startup_mission"
STARTUP_SCRIPT      = "/home/dexi/start_web.sh"
FLIGHT_MODE_FILE    = "/home/dexi/.flight_mode_led"
FLIGHT_MODE_CMD     = ("/usr/bin/python3 /home/dexi/dexi_ws/install/dexi_led/lib/dexi_led/"
                       "led_flight_mode_status --ros-args -r __node:=led_flight_mode_status "
                       "-r __ns:=/dexi")

def list_missions():
    try:
        files = sorted(f for f in os.listdir(MISSIONS_DIR)
                       if f.endswith(".py") and f not in BUILTIN)
        if DEFAULT_MISSION in files:
            files.remove(DEFAULT_MISSION)
            files.insert(0, DEFAULT_MISSION)
        return files
    except:
        return []

def do_run_mission(filename):
    global active_mission_proc, active_mission_name
    do_stop_mission()
    src = ("source /home/dexi/ros2_jazzy/install/setup.bash && "
           "source /home/dexi/dexi_ws/install/setup.bash")
    cmd = f'bash -c "{src} && cd {MISSIONS_DIR} && python3 {filename}"'
    with mission_lock:
        active_mission_proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
        active_mission_name = filename

PROTECTED = {"student_mission.py", "my_first_mission.py"}

# ── Flight mode LED helpers ───────────────────────────────────────────────────
def get_flight_mode_enabled():
    try:
        return open(FLIGHT_MODE_FILE).read().strip() != "disabled"
    except:
        return True

def set_flight_mode(enabled):
    with open(FLIGHT_MODE_FILE, "w") as f:
        f.write("enabled" if enabled else "disabled")
    if enabled:
        subprocess.Popen(f"sudo /bin/bash -c 'nohup {FLIGHT_MODE_CMD} &'", shell=True)
    else:
        subprocess.run(["sudo", "pkill", "-f", "led_flight_mode_status"])
    _write_start_web()

def _write_start_web():
    flight_enabled = get_flight_mode_enabled()
    startup_mission = get_startup_mission()
    lines = [
        "#!/bin/bash",
        "sleep 35",
        "source /home/dexi/ros2_jazzy/install/setup.bash",
        "source /home/dexi/dexi_ws/install/setup.bash",
        f"cd {MISSIONS_DIR}",
        "nohup python3 student_mission.py >> /tmp/student_mission.log 2>&1 &",
    ]
    if not flight_enabled:
        lines += ["sleep 5", "sudo pkill -f led_flight_mode_status"]
    if startup_mission and startup_mission != "student_mission.py":
        lines += [
            "sleep 5",
            f'MISSION=$(cat {STARTUP_FILE} 2>/dev/null | tr -d \'\\n\')',
            '[ -n "$MISSION" ] && [ "$MISSION" != "student_mission.py" ] && python3 "$MISSION" >> /tmp/startup_mission.log 2>&1 &',
        ]
    with open("/home/dexi/start_web.sh", "w") as f:
        f.write("\n".join(lines) + "\n")
    os.chmod("/home/dexi/start_web.sh", 0o755)

# ── Startup mission helpers ───────────────────────────────────────────────────
def get_startup_mission():
    try:
        with open(STARTUP_FILE) as f:
            return f.read().strip()
    except:
        return ""

def set_startup_mission(filename):
    with open(STARTUP_FILE, "w") as f:
        f.write(filename + "\n")
    _write_start_web()
    _update_crontab()

def clear_startup_mission():
    try: os.remove(STARTUP_FILE)
    except: pass
    _write_start_web()
    _update_crontab()

def _update_crontab():
    result = subprocess.run(["crontab", "-l"], capture_output=True, text=True)
    existing = result.stdout if result.returncode == 0 else ""
    lines = [l for l in existing.splitlines()
             if "start_web.sh" not in l and "run_startup_mission" not in l and l.strip()]
    lines.append(f"@reboot /bin/bash {STARTUP_SCRIPT}")
    new_crontab = "\n".join(lines) + "\n"
    subprocess.run(["crontab", "-"], input=new_crontab, text=True, capture_output=True)

def do_stop_mission():
    global active_mission_proc, active_mission_name
    name_to_kill = active_mission_name
    with mission_lock:
        if active_mission_proc and active_mission_proc.poll() is None:
            try:
                os.killpg(os.getpgid(active_mission_proc.pid), signal.SIGTERM)
                active_mission_proc.wait(timeout=2)
            except Exception:
                pass
            try:
                if active_mission_proc.poll() is None:
                    os.killpg(os.getpgid(active_mission_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        if name_to_kill and name_to_kill not in PROTECTED:
            subprocess.run(f"pkill -9 -f {name_to_kill}", shell=True)
        active_mission_proc = None
        active_mission_name = ""
    if node_ref:
        node_ref.set_led("white", force=True)



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
            mission_running = (active_mission_proc is not None
                               and active_mission_proc.poll() is None)
            body = json.dumps({
                "running":         program_running,
                "color":           detected_color,
                "color_enabled":   color_enabled,
                "mission_running": mission_running,
                "mission_name":    active_mission_name,
                "claw_busy":       claw_busy,
            }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/api/missions":
            body = json.dumps({"missions": list_missions()}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/api/flight_mode":
            body = json.dumps({"enabled": get_flight_mode_enabled()}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/api/startup":
            body = json.dumps({"mission": get_startup_mission() or None}).encode()
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
                    target=node_ref.open_claw, daemon=True).start()
            import pathlib
            pathlib.Path("/tmp/claw_open_signal").touch()
            resp = json.dumps({"ok": True}).encode()

        elif self.path == "/api/close_claw":
            if node_ref:
                threading.Thread(
                    target=node_ref.close_claw, daemon=True).start()
            resp = json.dumps({"ok": True}).encode()

        elif self.path == "/api/upload":
            student_name = body.get("student_name", "student")
            content      = body.get("content", "")
            safe = "".join(
                c if c.isalnum() or c == "_" else "_"
                for c in student_name.lower().replace(" ", "_")
            ).strip("_") or "student"
            filename = f"{safe}_mission.py"
            filepath = os.path.join(MISSIONS_DIR, filename)
            try:
                with open(filepath, "w") as fh:
                    fh.write(content)
                resp = json.dumps({"ok": True, "filename": filename}).encode()
            except Exception as e:
                resp = json.dumps({"ok": False, "error": str(e)}).encode()

        elif self.path == "/api/run_mission":
            filename = body.get("filename", "")
            if filename and filename in list_missions() and filename not in PROTECTED:
                threading.Thread(target=do_run_mission, args=(filename,), daemon=True).start()

        elif self.path == "/api/stop_mission":
            threading.Thread(target=do_stop_mission, daemon=True).start()

        elif self.path == "/api/set_startup":
            filename = body.get("filename", "")
            try:
                if filename and filename in list_missions() and filename not in PROTECTED:
                    set_startup_mission(filename)
                elif not filename:
                    clear_startup_mission()
                resp = json.dumps({"ok": True}).encode()
            except Exception as e:
                resp = json.dumps({"ok": False, "error": str(e)}).encode()

        elif self.path == "/api/clear_startup":
            try:
                clear_startup_mission()
                resp = json.dumps({"ok": True}).encode()
            except Exception as e:
                resp = json.dumps({"ok": False, "error": str(e)}).encode()

        elif self.path == "/api/flight_mode":
            enabled = body.get("enabled", True)
            try:
                threading.Thread(target=set_flight_mode, args=(enabled,), daemon=True).start()
                resp = json.dumps({"ok": True}).encode()
            except Exception as e:
                resp = json.dumps({"ok": False, "error": str(e)}).encode()

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

        self.create_subscription(CompressedImage, "/cam0/image_raw/compressed", self.cb, qos)
        self.set_led("white")
        self.send_servo(150)
        self.get_logger().info("Student Mission ready — open http://192.168.4.1:8081")

    def set_led(self, color, force=False):
        if self._led_color == color and not force:
            return
        req = LEDRingColor.Request()
        req.color = color
        self.led.call_async(req)
        self._led_color = color

    def open_claw(self):
      self.send_servo(OPEN_ANGLE) # 32

    def close_claw(self):
        self.send_servo(CLOSED_ANGLE) # 150
#-----------------------------------------------------------------------------------------------

    # ===============================================================
    # ~1: The Faulty code that is limited claw movements and is flipped,
    # ===============================================================


    def send_servo(self, angle: int):
        req = ServoControl.Request()
        req.pin    = CLAW_CHANNEL
        req.angle  = angle
        req.min_pw = 0
        req.max_pw = 0
        self.servo.call_async(req)

    def _claw_sequence2(self):
      global CLAW_LOCK
      try:
          self.send_servo(CLOSED_ANGLE)
          time.sleep(2.0)
          self.send_servo(OPEN_ANGLE)
      finally:
          CLAW_LOCK = False

    def _claw_sequence(self):
        global claw_busy
        try:
            # Close
            self.send_servo(CLOSED_ANGLE)
            time.sleep(MOVE_DURATION)
            self.send_servo(STOP_ANGLE)
            # Hold closed for 3 seconds
            time.sleep(15.0)
            # Open
            self.send_servo(OPEN_ANGLE)
            time.sleep(MOVE_DURATION)
            self.send_servo(STOP_ANGLE)
        finally:
            claw_busy = False


#-----------------------------------------------------------------------------------------------
    def cb(self, msg):
        global detected_color, latest_frame, claw_busy

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        if not program_running:
            self.set_led("white")
            detected_color = "none"
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
            with frame_lock: latest_frame = buf.tobytes()
            return

        # ── Color detection ───────────────────────────────────────────────────
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        found = None

        # Skip detection entirely while claw is busy
        if not claw_busy:
            for name, cfg in COLORS.items():
                if not color_enabled.get(name, True):
                    continue
                mask = cv2.inRange(hsv, np.array(cfg["lower"]), np.array(cfg["upper"]))
                if "lower2" in cfg:
                    mask = cv2.bitwise_or(mask,
                        cv2.inRange(hsv, np.array(cfg["lower2"]), np.array(cfg["upper2"])))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours and cv2.contourArea(max(contours, key=cv2.contourArea)) > 500:
                    found = name
                    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
                    cv2.rectangle(frame, (x, y), (x+w, y+h), cfg["bgr"], 2)
                    cv2.putText(frame, name, (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, cfg["bgr"], 2)
                    break

            # Blue triggers claw sequence
            if found == "blue":
                claw_busy = True
                threading.Thread(target=self._claw_sequence2, daemon=True).start()

        # ── Set LED ───────────────────────────────────────────────────────────
        detected_color = found or "none"
        self.set_led(found if found else "white")

        # ── Status overlay ────────────────────────────────────────────────────
        if claw_busy:
            text  = "CLAW SEQUENCE..."
            color = (255, 165, 0)
        elif found:
            text  = f"DETECTED: {found.upper()}"
            color = COLORS[found]["bgr"]
        else:
            text  = "Scanning..."
            color = (150, 150, 150)

        cv2.rectangle(frame, (0, 0), (frame.shape[1], 30), (15, 15, 15), -1)
        cv2.putText(frame, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        with frame_lock: latest_frame = buf.tobytes()


def main():
    _write_start_web()
    _update_crontab()

    server = ThreadingHTTPServer(("", 8081), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("Student Mission web page: http://192.168.4.1:8081")

    rclpy.init()
    rclpy.spin(StudentMission())
    rclpy.shutdown()






if __name__ == "__main__":
    main()
