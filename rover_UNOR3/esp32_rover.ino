/*
  ============================================================
  2-DOF Phone Holder + Rover Bridge — ESP32 [v10 FINAL]
  ============================================================
  BASE SERVO  → GPIO 18  (positional 0°–180°, smooth sweep)
  UPPER SERVO → GPIO 19  (closed-loop PID via MPU6050)
  MPU6050 SDA → GPIO 21
  MPU6050 SCL → GPIO 22
  UNO SERIAL  → GPIO 16 (RX) / GPIO 17 (TX)  ← rover bridge

  DASHBOARD FEATURES:
  ┌─ Mode ──────────────────────────────────────────────────┐
  │  Autonomous  /  Remote control                          │
  ├─ Base servo ────────────────────────────────────────────┤
  │  AUTO: sweeps CW↔CCW (wire-safe)                       │
  │  REMOTE: ↺ CCW / STOP / CW ↻ + speed slider           │
  ├─ Upper servo ───────────────────────────────────────────┤
  │  PID live data (angle, error, integral, pulse)          │
  │  FLIP button: toggles target 90° ↔ 180° instantly      │
  ├─ Rover drive (UNO bridge) ──────────────────────────────┤
  │  D-pad: F / B / L / R / STOP + speed slider            │
  │  Commands forwarded to UNO via Serial2                  │
  └─────────────────────────────────────────────────────────┘

  UNO expects these serial strings (115200 baud):
    "MODE:AUTO"    "MODE:REMOTE"
    "F:170"        "B:170"    "L:170"    "R:170"    "S:170"

  PHONE: Realme 12x 5G — 188g
  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ─── WiFi CREDENTIALS ─────────────────────────────────────
#define AP_SSID   "PhoneHolder"
#define AP_PASS   "holder1234"
#define STA_SSID  "realme 12x 5G"
#define STA_PASS  "1234567890"

// ─── PINS ─────────────────────────────────────────────────
#define BASE_SERVO_PIN   18
#define UPPER_SERVO_PIN  19
#define SDA_PIN          21
#define SCL_PIN          22
#define UNO_RX_PIN       16   // ESP32 RX ← UNO TX
#define UNO_TX_PIN       17   // ESP32 TX → UNO RX

// ─── SERIAL TO UNO ────────────────────────────────────────
#define UNO_SERIAL Serial2
#define UNO_BAUD   115200

// ─── SERVO PULSE RANGE ────────────────────────────────────
#define SERVO_MIN_US    620
#define SERVO_MAX_US    2380
#define SERVO_CENTER_US 1500

// ─── BASE SERVO CONFIG ────────────────────────────────────
// Standard MG995R positional servo — 0° to 180° physical range.
// We sweep BASE_MIN_DEG to BASE_MAX_DEG to stay away from hard stops.
// Easing: in the last EASE_ZONE degrees before each end, step size
// halves — so reversal feels smooth instead of a hard slam.
#define BASE_MIN_DEG    15      // safe min — clear of hard stop
#define BASE_MAX_DEG    165     // safe max — clear of hard stop
#define BASE_STEP_MS    18      // ms between each degree step (~55Hz)
#define BASE_SPEED_DEF  1       // degrees per step (1=smooth)
#define EASE_ZONE       20      // degrees near ends where speed halves
#define STOP_PAUSE_MS   300     // ms pause at each end before reversing
#define REMOTE_STEP_DEG 1       // degrees per step in remote mode

enum BaseState { BASE_SWEEP, BASE_PAUSE };

// ─── UPPER SERVO / PID CONFIG (UNCHANGED) ─────────────────
float Kp = 2.0;
float Ki = 0.12;
float Kd = 1.0;

// flipTarget: normally 90.0, becomes 180.0 when flipped
// PID runs against this variable — flip = instant retarget
float targetAngle  = 90.0;
bool  flipped      = false;    // false=90°, true=180°

const float PID_DEADBAND  = 0.9;
const int   PID_LOOP_MS   = 15;
const float I_MAX         =  200.0;
const float I_MIN         = -200.0;
const float OUT_MAX_US    =  300.0;
const float OUT_MIN_US    = -300.0;
const int   UPPER_PULSE_MIN = 1200;
const int   UPPER_PULSE_MAX = 1800;

// When flipped to 180° the pulse limits open up to allow full range
// 180° on MG995R ≈ SERVO_MAX_US — guard keeps it safe
const int   UPPER_PULSE_MIN_FLIP = 1200;
const int   UPPER_PULSE_MAX_FLIP = 2200;

#define GET_ANGLE()   mpu.getAngleX()
#define AXIS_SIGN     1
#define ANGLE_OFFSET  0.0

// ─── OBJECTS ──────────────────────────────────────────────
Servo            baseServo;
Servo            upperServo;
MPU6050          mpu(Wire);
WebServer        server(80);
WebSocketsServer webSocket(81);

// ─── STATE ────────────────────────────────────────────────
float pidIntegral  = 0.0;
float pidLastError = 0.0;
bool  mpuReady     = false;

unsigned long lastPidTime   = 0;
unsigned long lastMpuUpdate = 0;

BaseState     baseState      = BASE_SWEEP;
unsigned long baseStateStart = 0;
unsigned long lastBaseStep   = 0;
bool          remoteMode     = false;
int           baseSpeed      = BASE_SPEED_DEF;  // degrees per step
String        baseCmdStr     = "S";
float         baseAngle      = BASE_MIN_DEG;     // current position (float for sub-step easing)
int           baseDir        = 1;                // +1 = toward MAX, -1 = toward MIN

int  driveSpeed = 170;
bool apMode     = false;

// ─── DASHBOARD HTML ───────────────────────────────────────
const char DASHBOARD[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Holder + Rover</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
:root{
  --bg:#0f0f0f;--card:#1a1a1a;--border:#2a2a2a;
  --teal:#1D9E75;--blue:#378ADD;--red:#E24B4A;--amber:#EF9F27;--purple:#7F77DD;
  --text:#e8e8e8;--muted:#666;
}
body{background:var(--bg);color:var(--text);font-family:system-ui,sans-serif;
  min-height:100vh;padding:12px;user-select:none}
.grid{display:grid;gap:12px;grid-template-columns:1fr}
@media(min-width:540px){.grid{grid-template-columns:1fr 1fr}.span2{grid-column:span 2}}
.card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px}
.card h3{font-size:11px;font-weight:500;color:var(--muted);text-transform:uppercase;
  letter-spacing:.08em;margin-bottom:12px}

/* status */
.status-bar{display:flex;align-items:center;justify-content:space-between;
  margin-bottom:12px;padding:10px 14px;background:var(--card);
  border-radius:10px;border:1px solid var(--border)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--red);transition:background .3s}
.dot.on{background:var(--teal)}
.status-text{font-size:13px;color:var(--muted)}
.mode-badge{font-size:12px;font-weight:500;padding:3px 10px;border-radius:8px;
  background:#222;color:var(--muted)}
.mode-badge.remote{background:#0d1a2d;color:var(--blue)}

/* toggle */
.toggle-row{display:flex;gap:8px}
.mode-btn{flex:1;padding:12px;border:none;border-radius:10px;font-size:14px;
  font-weight:500;cursor:pointer;transition:all .2s}
#btn-auto{background:#1a2a1a;color:var(--teal);border:1px solid var(--teal)}
#btn-auto.active{background:var(--teal);color:#fff}
#btn-remote{background:#1a1a2a;color:var(--blue);border:1px solid var(--blue)}
#btn-remote.active{background:var(--blue);color:#fff}

/* base dir buttons */
.dir-row{display:flex;gap:8px;margin-bottom:14px}
.dir-btn{flex:1;padding:14px 8px;border-radius:10px;font-size:22px;cursor:pointer;
  transition:all .15s;border:1px solid var(--border);background:#222;color:var(--text)}
.dir-btn:active,.dir-btn.pressed{background:var(--blue);border-color:var(--blue);color:#fff}
#btn-stop-base{font-size:13px;font-weight:500;color:var(--muted);background:#1a1a1a}
#btn-stop-base.pressed{background:var(--red);border-color:var(--red);color:#fff}

/* d-pad rover */
.dpad{display:grid;grid-template-columns:repeat(3,68px);
  grid-template-rows:repeat(3,68px);gap:8px;margin:0 auto 14px;width:max-content}
.dpad-btn{width:68px;height:68px;border-radius:12px;border:1px solid var(--border);
  background:#222;color:var(--text);font-size:24px;cursor:pointer;
  display:flex;align-items:center;justify-content:center;transition:background .1s}
.dpad-btn:active,.dpad-btn.pressed{background:var(--teal);border-color:var(--teal)}
.dpad-stop{font-size:12px;font-weight:500;color:var(--muted);background:#1a1a1a}

/* flip button */
.flip-btn{width:100%;padding:13px;border:none;border-radius:10px;font-size:14px;
  font-weight:500;cursor:pointer;transition:all .2s;margin-bottom:12px;
  background:#1a1a2a;color:var(--purple);border:1px solid var(--purple)}
.flip-btn.flipped{background:var(--purple);color:#fff}

/* sliders */
.slider-group{margin-bottom:4px}
.slider-label{display:flex;justify-content:space-between;font-size:13px;
  margin-bottom:6px;color:var(--muted)}
.slider-label span:last-child{color:var(--text);font-weight:500}
input[type=range]{width:100%;height:6px;-webkit-appearance:none;
  background:#2a2a2a;border-radius:3px;outline:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;
  border-radius:50%;background:var(--amber);cursor:pointer}
input[type=range].blue::-webkit-slider-thumb{background:var(--blue)}
input[type=range].teal::-webkit-slider-thumb{background:var(--teal)}

/* PID data */
.pid-row{display:flex;justify-content:space-between;font-size:13px;
  padding:6px 0;border-bottom:1px solid var(--border)}
.pid-row:last-child{border:none}
.pid-row .k{color:var(--muted)}
.pid-row .v{font-weight:500;font-variant-numeric:tabular-nums;color:var(--teal)}
.pid-row .v.warn{color:var(--amber)}
.pid-row .v.flipped-tag{color:var(--purple)}

/* info */
.info-row{display:flex;justify-content:space-between;font-size:13px;
  padding:6px 0;border-bottom:1px solid var(--border)}
.info-row:last-child{border:none}
.info-row .key{color:var(--muted)}
.info-row .val{font-weight:500}
.note{font-size:11px;color:var(--muted);margin-top:8px}
</style>
</head><body>

<div class="status-bar">
  <div style="display:flex;align-items:center;gap:8px">
    <div class="dot" id="ws-dot"></div>
    <span class="status-text" id="ws-text">Connecting…</span>
  </div>
  <span class="mode-badge" id="mode-badge">AUTO</span>
</div>

<div class="grid">

  <!-- ── Mode ── -->
  <div class="card span2">
    <h3>Mode</h3>
    <div class="toggle-row">
      <button class="mode-btn active" id="btn-auto"   onclick="setMode('AUTO')">Autonomous</button>
      <button class="mode-btn"        id="btn-remote" onclick="setMode('REMOTE')">Remote control</button>
    </div>
  </div>

  <!-- ── Base servo ── -->
  <div class="card">
    <h3>Base servo (360°)</h3>
    <div class="dir-row">
      <button class="dir-btn" id="btn-ccw"
        ontouchstart="baseDir('CCW')" ontouchend="baseDir('S')"
        onmousedown="baseDir('CCW')"  onmouseup="baseDir('S')">↺</button>
      <button class="dir-btn" id="btn-stop-base" onclick="baseDir('S')">STOP</button>
      <button class="dir-btn" id="btn-cw"
        ontouchstart="baseDir('CW')" ontouchend="baseDir('S')"
        onmousedown="baseDir('CW')"  onmouseup="baseDir('S')">↻</button>
    </div>
    <div class="slider-group">
      <div class="slider-label"><span>Speed</span><span id="base-spd-val">200</span></div>
      <input type="range" class="blue" min="50" max="400" value="200"
        oninput="setBaseSpeed(this.value)">
    </div>
    <p class="note" id="base-note">Switch to remote to control manually.</p>
  </div>

  <!-- ── Upper servo ── -->
  <div class="card">
    <h3>Upper servo (PID)</h3>
    <button class="flip-btn" id="flip-btn" onclick="flipTarget()">
      ⟳ Flip: 90° → 180°
    </button>
    <div class="pid-row"><span class="k">Target</span>   <span class="v" id="pid-target">90°</span></div>
    <div class="pid-row"><span class="k">Actual</span>   <span class="v" id="pid-angle">--°</span></div>
    <div class="pid-row"><span class="k">Error</span>    <span class="v" id="pid-err">--°</span></div>
    <div class="pid-row"><span class="k">Integral</span> <span class="v" id="pid-i">--</span></div>
    <div class="pid-row"><span class="k">Pulse</span>    <span class="v" id="pid-pulse">-- µs</span></div>
  </div>

  <!-- ── Rover drive ── -->
  <div class="card">
    <h3>Rover drive (UNO)</h3>
    <div class="dpad">
      <div></div>
      <button class="dpad-btn" id="dpF"
        ontouchstart="drive('F')" ontouchend="drive('S')"
        onmousedown="drive('F')" onmouseup="drive('S')">▲</button>
      <div></div>
      <button class="dpad-btn" id="dpL"
        ontouchstart="drive('L')" ontouchend="drive('S')"
        onmousedown="drive('L')" onmouseup="drive('S')">◀</button>
      <button class="dpad-btn dpad-stop" onclick="drive('S')">STOP</button>
      <button class="dpad-btn" id="dpR"
        ontouchstart="drive('R')" ontouchend="drive('S')"
        onmousedown="drive('R')" onmouseup="drive('S')">▶</button>
      <div></div>
      <button class="dpad-btn" id="dpB"
        ontouchstart="drive('B')" ontouchend="drive('S')"
        onmousedown="drive('B')" onmouseup="drive('S')">▼</button>
      <div></div>
    </div>
    <div class="slider-group">
      <div class="slider-label"><span>Drive speed</span><span id="drv-spd-val">170</span></div>
      <input type="range" class="teal" min="80" max="255" value="170"
        oninput="setDriveSpeed(this.value)">
    </div>
    <p class="note">Active in remote mode only. Hold to move.</p>
  </div>

  <!-- ── Connection info ── -->
  <div class="card span2">
    <h3>Connection info</h3>
    <div class="info-row"><span class="key">Network</span>   <span class="val" id="info-ssid">--</span></div>
    <div class="info-row"><span class="key">IP address</span><span class="val" id="info-ip">--</span></div>
    <div class="info-row"><span class="key">WiFi mode</span> <span class="val" id="info-wmode">--</span></div>
  </div>

</div>

<script>
let ws;
let currentMode  = 'AUTO';
let driveSpeed   = 170;
let isFlipped    = false;

function connect() {
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.onopen = () => {
    document.getElementById('ws-dot').classList.add('on');
    document.getElementById('ws-text').textContent = 'Connected';
    ws.send('GET_INFO');
  };
  ws.onclose = () => {
    document.getElementById('ws-dot').classList.remove('on');
    document.getElementById('ws-text').textContent = 'Disconnected — retrying…';
    setTimeout(connect, 2000);
  };
  ws.onmessage = e => {
    try {
      const d = JSON.parse(e.data);
      if (d.type === 'info') updateInfo(d);
      if (d.type === 'pid')  updatePID(d);
    } catch(err) {}
  };
}

function send(msg) { if (ws && ws.readyState === 1) ws.send(msg); }

function setMode(m) {
  currentMode = m;
  send('MODE:' + m);
  document.getElementById('btn-auto').classList.toggle('active',   m === 'AUTO');
  document.getElementById('btn-remote').classList.toggle('active', m === 'REMOTE');
  const badge = document.getElementById('mode-badge');
  badge.textContent = m;
  badge.className = 'mode-badge' + (m === 'REMOTE' ? ' remote' : '');
  document.getElementById('base-note').textContent =
    m === 'REMOTE' ? 'Hold button to rotate.' : 'Switch to remote to control manually.';
}

// ── Base servo ────────────────────────────────────────────
function baseDir(cmd) {
  if (currentMode !== 'REMOTE') return;
  send('BASE:' + cmd);
  document.getElementById('btn-ccw').classList.toggle('pressed', cmd === 'CCW');
  document.getElementById('btn-cw').classList.toggle('pressed',  cmd === 'CW');
  document.getElementById('btn-stop-base').classList.toggle('pressed', cmd === 'S');
}
function setBaseSpeed(v) {
  document.getElementById('base-spd-val').textContent = v;
  send('BASE_SPEED:' + v);
}

// ── Upper servo flip ──────────────────────────────────────
function flipTarget() {
  isFlipped = !isFlipped;
  send('FLIP:' + (isFlipped ? '180' : '90'));
  const btn = document.getElementById('flip-btn');
  btn.classList.toggle('flipped', isFlipped);
  btn.textContent = isFlipped ? '⟳ Flip: 180° → 90°' : '⟳ Flip: 90° → 180°';
  document.getElementById('pid-target').textContent = isFlipped ? '180°' : '90°';
}

// ── Rover drive ───────────────────────────────────────────
function drive(cmd) {
  if (currentMode !== 'REMOTE') return;
  send('DRIVE:' + cmd + ':' + driveSpeed);
  ['F','B','L','R'].forEach(d => {
    document.getElementById('dp'+d).classList.toggle('pressed', cmd === d);
  });
}
function setDriveSpeed(v) {
  driveSpeed = parseInt(v);
  document.getElementById('drv-spd-val').textContent = v;
}

// ── PID display ───────────────────────────────────────────
function updatePID(d) {
  document.getElementById('pid-angle').textContent = d.angle + '°';
  const errEl = document.getElementById('pid-err');
  errEl.textContent = d.err + '°';
  errEl.className = 'v' + (Math.abs(parseFloat(d.err)) > 5 ? ' warn' : '');
  document.getElementById('pid-i').textContent     = d.i;
  document.getElementById('pid-pulse').textContent = d.pulse + ' µs';
}

function updateInfo(d) {
  document.getElementById('info-ssid').textContent  = d.ssid  || '--';
  document.getElementById('info-ip').textContent    = d.ip    || '--';
  document.getElementById('info-wmode').textContent = d.wmode || '--';
}

connect();
</script>
</body></html>
)rawhtml";

// ─── HELPERS ──────────────────────────────────────────────
void writeBaseAngle(float angle) {
  angle = constrain(angle, BASE_MIN_DEG, BASE_MAX_DEG);
  baseServo.write((int)round(angle));
}

float readAngle() {
  return (AXIS_SIGN * GET_ANGLE()) + 90.0 + ANGLE_OFFSET;
}

// ─── PID (UPPER SERVO — CORE LOGIC UNCHANGED) ─────────────
// Only change: uses 'targetAngle' variable instead of
// hardcoded 90.0, so FLIP:180 just changes that variable.
float computePID(float actual, float dt) {
  float error = targetAngle - actual;

  if (abs(error) < PID_DEADBAND) {
    pidLastError = error;
    return (Ki * pidIntegral);
  }

  pidIntegral += error * dt;
  pidIntegral  = constrain(pidIntegral, I_MIN, I_MAX);

  float derivative = (error - pidLastError) / dt;
  pidLastError = error;

  float output = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);
  return constrain(output, OUT_MIN_US, OUT_MAX_US);
}

// ─── MPU INIT (UNCHANGED) ─────────────────────────────────
bool initMPU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  byte status = mpu.begin();
  if (status != 0) {
    Serial.printf("[MPU] ERROR %d — check wiring\n", status);
    return false;
  }
  Serial.println("[MPU] Found. Calibrating — keep still 3s...");
  delay(500);
  mpu.calcOffsets();
  Serial.println("[MPU] Done.");
  return true;
}

// ─── UPPER SERVO BOOT (UNCHANGED) ─────────────────────────
void bootUpperServo() {
  for (int i = 0; i < 10; i++) {
    upperServo.writeMicroseconds(SERVO_CENTER_US);
    delay(50);
  }
  for (int a = 60; a <= 90; a++) {
    upperServo.writeMicroseconds(map(a, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
    delay(25);
  }
  for (int i = 0; i < 5; i++) {
    upperServo.writeMicroseconds(SERVO_CENTER_US);
    delay(80);
  }
  pidIntegral = 0;
  Serial.println("[UPPER] At 90°. PID running.");
}

// ─── WiFi ─────────────────────────────────────────────────
void initWiFi() {
  Serial.printf("[WiFi] Trying: %s\n", STA_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 8000) {
    delay(300); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    apMode = false;
    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    apMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] AP: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }
}

// ─── WEB SERVER ───────────────────────────────────────────
void initServer() {
  server.on("/", []() {
    server.sendHeader("Content-Encoding", "identity");
    server.send_P(200, "text/html", DASHBOARD);
  });
  server.begin();
}

// ─── WEBSOCKET HANDLER ────────────────────────────────────
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type != WStype_TEXT) return;
  String msg = String((char*)payload);

  // ── GET_INFO ──────────────────────────────────────────
  if (msg == "GET_INFO") {
    String ip   = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    String ssid = apMode ? AP_SSID : STA_SSID;
    String wm   = apMode ? "Hotspot (AP)" : "Home WiFi (STA)";
    webSocket.sendTXT(num,
      "{\"type\":\"info\",\"ssid\":\"" + ssid +
      "\",\"ip\":\"" + ip +
      "\",\"wmode\":\"" + wm + "\"}");
    return;
  }

  // ── MODE ──────────────────────────────────────────────
  if (msg.startsWith("MODE:")) {
    String m = msg.substring(5);
    remoteMode = (m == "REMOTE");
    if (!remoteMode) {
      // Back to AUTO — hold current angle, resume sweep from here
      baseCmdStr     = "S";
      baseState      = BASE_SWEEP;
      baseStateStart = millis();
      lastBaseStep   = millis();
    }
    // Forward mode to UNO rover
    UNO_SERIAL.println(remoteMode ? "MODE:REMOTE" : "MODE:AUTO");
    Serial.printf("[MODE] %s\n", m.c_str());
    return;
  }

  // ── BASE servo ────────────────────────────────────────
  if (msg.startsWith("BASE:") && remoteMode) {
    baseCmdStr = msg.substring(5);   // "CW", "CCW", "S"
    return;
  }

  if (msg.startsWith("BASE_SPEED:")) {
    // Speed slider now maps to degrees-per-step: 1 (slow) to 5 (fast)
    int raw = msg.substring(11).toInt();
    baseSpeed = constrain(map(raw, 50, 400, 1, 5), 1, 5);
    return;
  }

  // ── FLIP upper servo target ───────────────────────────
  // Dashboard sends "FLIP:90" or "FLIP:180"
  // We just change targetAngle — PID does the rest automatically
  if (msg.startsWith("FLIP:")) {
    int newTarget = msg.substring(5).toInt();
    targetAngle = (float)constrain(newTarget, 0, 180);
    flipped     = (newTarget == 180);
    // Reset integral so PID doesn't wind up fighting the flip
    pidIntegral = 0;
    pidLastError = 0;
    Serial.printf("[FLIP] Target now %.0f°\n", targetAngle);
    return;
  }

  // ── DRIVE (rover → UNO) ───────────────────────────────
  // Format from dashboard: "DRIVE:F:170"
  // Forwarded to UNO as:   "F:170"
  if (msg.startsWith("DRIVE:") && remoteMode) {
    int c1  = msg.indexOf(':');
    int c2  = msg.indexOf(':', c1 + 1);
    String cmd = msg.substring(c1 + 1, c2);
    int    spd = msg.substring(c2 + 1).toInt();
    UNO_SERIAL.println(cmd + ":" + String(spd));
    return;
  }
}

// ─── BASE AUTO SWEEP ──────────────────────────────────────
// Moves baseAngle by baseSpeed degrees every BASE_STEP_MS ms.
// Near each end (within EASE_ZONE degrees) the step is halved
// so reversal is smooth — no slamming into the hard stop.
void runBaseAuto() {
  unsigned long now = millis();

  if (baseState == BASE_PAUSE) {
    if (now - baseStateStart >= STOP_PAUSE_MS) {
      baseState = BASE_SWEEP;
      Serial.printf("[BASE] Resume sweep dir=%d\n", baseDir);
    }
    return;
  }

  // BASE_SWEEP
  if (now - lastBaseStep < BASE_STEP_MS) return;
  lastBaseStep = now;

  // Easing: near the ends, halve the effective step size
  float distToEnd = (baseDir > 0)
    ? (BASE_MAX_DEG - baseAngle)
    : (baseAngle - BASE_MIN_DEG);

  float step = (distToEnd < EASE_ZONE)
    ? max(0.5f, baseSpeed * 0.5f)
    : (float)baseSpeed;

  baseAngle += baseDir * step;

  // Hit an end → clamp, pause, reverse
  if (baseAngle >= BASE_MAX_DEG) {
    baseAngle  = BASE_MAX_DEG;
    baseDir    = -1;
    baseState  = BASE_PAUSE;
    baseStateStart = now;
    Serial.println("[BASE] MAX → pausing");
  } else if (baseAngle <= BASE_MIN_DEG) {
    baseAngle  = BASE_MIN_DEG;
    baseDir    = +1;
    baseState  = BASE_PAUSE;
    baseStateStart = now;
    Serial.println("[BASE] MIN → pausing");
  }

  writeBaseAngle(baseAngle);
}

// ─── BASE REMOTE ──────────────────────────────────────────
// CW/CCW buttons nudge baseAngle by REMOTE_STEP_DEG each loop tick.
// Speed slider on dashboard changes baseSpeed (steps per tick).
void runBaseRemote() {
  unsigned long now = millis();
  if (now - lastBaseStep < BASE_STEP_MS) return;
  lastBaseStep = now;

  if      (baseCmdStr == "CW")  baseAngle = constrain(baseAngle + baseSpeed, BASE_MIN_DEG, BASE_MAX_DEG);
  else if (baseCmdStr == "CCW") baseAngle = constrain(baseAngle - baseSpeed, BASE_MIN_DEG, BASE_MAX_DEG);
  // "S" = hold position — no change to baseAngle

  writeBaseAngle(baseAngle);
}

// ─── SETUP ────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== 2-DOF Holder + Rover v10 ===");

  // UNO serial bridge
  UNO_SERIAL.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
  Serial.println("[UNO] Serial2 ready on GPIO16/17");

  // MPU first — before servo vibration
  mpuReady = initMPU();
  if (!mpuReady) Serial.println("[WARN] MPU missing — upper servo open-loop");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  baseServo.setPeriodHertz(50);
  upperServo.setPeriodHertz(50);
  baseServo.attach(BASE_SERVO_PIN,   SERVO_MIN_US, SERVO_MAX_US);
  delay(100);
  upperServo.attach(UPPER_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  delay(100);

  writeBaseAngle(BASE_MIN_DEG);
  delay(300);
  bootUpperServo();

  initWiFi();
  initServer();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  baseStateStart = millis();
  lastBaseStep   = millis();
  lastPidTime    = millis();
  lastMpuUpdate  = millis();

  Serial.println("[BOOT] Ready. Open: http://" +
    (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()));
}

// ─── LOOP ─────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  webSocket.loop();
  server.handleClient();

  // ── MPU: 200Hz ───────────────────────────────────────────
  if (now - lastMpuUpdate >= 5) {
    lastMpuUpdate = now;
    if (mpuReady) mpu.update();
  }

  // ── UPPER SERVO PID: 67Hz ────────────────────────────────
  // Core logic 100% unchanged from v8.
  // Only difference: TARGET_ANGLE → targetAngle (flip support)
  if (now - lastPidTime >= PID_LOOP_MS) {
    float dt    = (now - lastPidTime) / 1000.0;
    lastPidTime = now;

    float actual     = mpuReady ? readAngle() : targetAngle;
    float correction = computePID(actual, dt);

    // Pulse limits widen when flipped to 180° to allow full travel
    int pMin = flipped ? UPPER_PULSE_MIN_FLIP : UPPER_PULSE_MIN;
    int pMax = flipped ? UPPER_PULSE_MAX_FLIP : UPPER_PULSE_MAX;
    int pulseUs = constrain(SERVO_CENTER_US + (int)correction, pMin, pMax);
    upperServo.writeMicroseconds(pulseUs);

    // Broadcast to dashboard + Serial every 300ms
    static int dbg = 0;
    if (++dbg >= 20) {
      dbg = 0;
      float err = targetAngle - actual;
      Serial.printf("[PID] target=%.0f° actual=%.1f° err=%.1f° I=%.1f corr=%d pulse=%d\n",
                    targetAngle, actual, err, pidIntegral, (int)correction, pulseUs);

      String json = "{\"type\":\"pid\","
                    "\"angle\":"  + String(actual, 1) +
                    ",\"err\":"   + String(err, 1) +
                    ",\"i\":"     + String(pidIntegral, 1) +
                    ",\"pulse\":" + String(pulseUs) + "}";
      webSocket.broadcastTXT(json);
    }
  }

  // ── BASE SERVO ────────────────────────────────────────────
  remoteMode ? runBaseRemote() : runBaseAuto();
}

/*
  ════════════════════════════════════════════════════════════
  LIBRARIES  (Arduino IDE → Manage Libraries)
  ════════════════════════════════════════════════════════════
  → ESP32Servo        by Kevin Harrington
  → MPU6050_light     by rfetick
  → WebSockets        by Markus Sattler

  ════════════════════════════════════════════════════════════
  WIRING
  ════════════════════════════════════════════════════════════
  MPU6050:
    VCC → ESP32 3.3V   GND → GND
    SDA → GPIO 21      SCL → GPIO 22
    AD0 → GND  (I2C addr 0x68)

  Servos (external 5V — never from ESP32 3.3V pin):
    5V supply (+) → both servo VCC
    5V supply (−) → both servo GND + ESP32 GND
    GPIO 18 → base servo signal
    GPIO 19 → upper servo signal

  UNO bridge:
    ESP32 GPIO 17 (TX) → UNO RX  (via 1kΩ resistor)
    ESP32 GPIO 16 (RX) → UNO TX
    ESP32 GND          → UNO GND   ← common ground required

  ════════════════════════════════════════════════════════════
  FLIP BUTTON NOTES
  ════════════════════════════════════════════════════════════
  Pressing FLIP on dashboard sends "FLIP:180" (or "FLIP:90").
  ESP32 changes targetAngle and resets PID integral/error.
  PID then drives the servo to the new target automatically.
  No manual intervention needed — sensor closes the loop.

  If 180° position looks slightly off, adjust ANGLE_OFFSET.

  ════════════════════════════════════════════════════════════
  BASE SERVO TUNING
  ════════════════════════════════════════════════════════════
  Standard MG995R positional servo. Sweeps 15° ↔ 165°.
  Hard stops at 0° and 180° are NEVER reached — no buzzing.

  BASE_STEP_MS  = how often angle updates (18ms = ~55Hz)
  BASE_SPEED_DEF = degrees per step (1 = smoothest, 5 = fast)
  EASE_ZONE     = degrees near each end where speed halves
  STOP_PAUSE_MS = pause at each end before reversing

  Dashboard speed slider maps 50–400 → 1–5 deg/step.

  BASE_MIN_DEG / BASE_MAX_DEG: tighten these if your specific
  MG995R still buzzes at the ends (try 20 / 160).

  ════════════════════════════════════════════════════════════
  UNO EXPECTS (115200 baud on its hardware Serial or Serial1)
  ════════════════════════════════════════════════════════════
  "MODE:AUTO"    → start autonomous obstacle avoidance
  "MODE:REMOTE"  → stop auto, wait for drive commands
  "F:170"        → forward  at speed 170
  "B:170"        → backward at speed 170
  "L:170"        → left     at speed 170
  "R:170"        → right    at speed 170
  "S:170"        → stop
  ════════════════════════════════════════════════════════════
*/
