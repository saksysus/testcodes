/*
   TRINETRA — MOVING CAR (ESP-NOW receiver + live web dashboard)
   ============================================================
   Receives
   - Track node 1  : "Curve 17 Ahead. SLOW DOWN" and "Slope Ahead"
   - Stationary dumper (TFT + LiDAR): "MOVE LEFT"
   Shows all of it on a web page hosted by this board.

   How to open the page
   1. Power the car board.
   2. On a phone/laptop join Wi-Fi  TRINETRA-CAR1  (password: trinetra123)
      (if the phone says "no internet", choose "stay connected")
   3. Open  http://192.168.4.1

   Also
   - Broadcasts a 1 Hz heartbeat so the stationary dumper can show
     "CAR LINK: CONNECTED" and measure signal strength.
   - Wi-Fi channel is fixed to 1 for the access point, which keeps ESP-NOW
     on channel 1 like every other node.
   - Serial prints this board's Wi-Fi MAC at boot. It must be
     00:70:07:E1:BC:EC — that is the address both senders use.

   Packet format is shared by every node:
     { uint8_t messageType; char message[64]; }
     1 = Curve, 2 = Slope, 3 = dumper command, 4 = car heartbeat, 5 = node heartbeat
   ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================
// SETTINGS
// ============================================================
#define WIFI_CHANNEL 1
const char* AP_SSID = "TRINETRA-CAR1";
const char* AP_PASS = "trinetra123";      // 8+ characters

#define DUMPER_ACTIVE_MS  2500   // command counts as "live" this long after the last packet
#define TRACK_ACTIVE_MS   8000   // track alerts are sent once per pass, so keep them longer
#define HEARTBEAT_MS      1000
#define LOG_SIZE             8

// ============================================================
// SHARED PACKET
// ============================================================
#define MSG_CURVE          1
#define MSG_SLOPE          2
#define MSG_DUMPER_CMD     3
#define MSG_CAR_HEARTBEAT  4
#define MSG_NODE_HEARTBEAT 5

// Stationary-node command expected by this car receiver.
#define MOVE_LEFT_COMMAND "MOVE LEFT"

typedef struct {
  uint8_t messageType;
  char message[64];
} AlertPacket;

uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ============================================================
// EVENT STORE (written by the ESP-NOW callback, read by the web handler)
// ============================================================
enum Src : uint8_t { SRC_TRACK = 0, SRC_DUMPER = 1 };

struct Event {
  uint32_t ms;        // millis() when last received, 0 = never
  uint8_t  src;
  uint8_t  type;
  uint16_t count;
  uint32_t seq;       // increments only when a genuinely new alert is created
  char     text[64];
};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Event lastDumper = {};
Event lastTrack  = {};
Event logBuf[LOG_SIZE];
uint8_t logNext = 0;
uint8_t logCount = 0;
uint32_t eventSeq = 0;

WebServer server(80);
unsigned long lastHeartbeat = 0;

// ============================================================
// NODE LINK STATUS
// ============================================================
volatile unsigned long lastStationaryHeartbeat = 0;
volatile unsigned long lastTrackHeartbeat = 0;
volatile int stationaryRssi = 0;
volatile int trackRssi = 0;

#define NODE_LINK_TIMEOUT_MS 3500UL

static bool stationaryLinked() {
  unsigned long t = lastStationaryHeartbeat;
  return t != 0 && (millis() - t <= NODE_LINK_TIMEOUT_MS);
}

static bool trackLinked() {
  unsigned long t = lastTrackHeartbeat;
  return t != 0 && (millis() - t <= NODE_LINK_TIMEOUT_MS);
}

// ============================================================
// WEB MOTOR CONTROL — L298N / 4-WHEEL DRIVE
// ============================================================
// Left side:  ENA=25, IN1=26, IN2=27  (two motors on OUT1/OUT2)
// Right side: ENB=14, IN3=18, IN4=19  (two motors on OUT3/OUT4)
// Hold a web button to move. Releasing the button sends STOP.
#define LEFT_EN   25
#define LEFT_IN1  26
#define LEFT_IN2  27
#define RIGHT_EN  14
#define RIGHT_IN1 18
#define RIGHT_IN2 19

#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RES  8
#define DRIVE_FAILSAFE_MS 700

// Set either side true if that side is physically reversed on your chassis.
#define LEFT_MOTOR_REVERSED  false
#define RIGHT_MOTOR_REVERSED true

enum DriveCommand : uint8_t { DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACK, DRIVE_LEFT, DRIVE_RIGHT };
DriveCommand currentDrive = DRIVE_STOP;
uint8_t currentSpeed = 180;
unsigned long lastDriveCommandMs = 0;

void stopMotors();
void applyMotorSide(int en, int in1, int in2, int direction, uint8_t pwm, bool reversed);
void setDrive(DriveCommand cmd, uint8_t pwm);

// ============================================================
// WEB PAGE (no external files — the car's Wi-Fi has no internet)
// ============================================================
const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#07090d">
<title>TRINETRA // CAR 1</title>
<style>
:root{
  --bg:#07090d;--panel:#0d1117;--panel2:#10161e;--line:#202a34;
  --text:#f1f6fa;--muted:#718291;--cyan:#51e8ff;--red:#ff3d56;
  --amber:#ffb83f;--green:#42eb8b;--orange:#ff713f;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;min-height:100%;background:
radial-gradient(circle at 20% 0,rgba(81,232,255,.08),transparent 28%),
radial-gradient(circle at 90% 10%,rgba(255,113,63,.08),transparent 25%),
var(--bg);color:var(--text);font-family:Inter,system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif}
body{overflow-x:hidden}
body:before{content:"";position:fixed;inset:0;pointer-events:none;opacity:.045;
background-image:linear-gradient(rgba(255,255,255,.12) 1px,transparent 1px),linear-gradient(90deg,rgba(255,255,255,.12) 1px,transparent 1px);
background-size:26px 26px}
main{width:min(980px,100%);margin:auto;padding:14px 12px 36px;position:relative}
.top{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:12px}
.brand{display:flex;gap:11px;align-items:center}
.eye{width:44px;height:28px;filter:drop-shadow(0 0 10px rgba(81,232,255,.5))}
.eye svg{width:100%;height:100%}
.title{font-size:22px;line-height:1;font-weight:950;letter-spacing:.2em}
.sub{margin-top:4px;font-size:9px;color:var(--muted);letter-spacing:.18em}
.link{display:flex;align-items:center;gap:7px;padding:8px 10px;border:1px solid var(--line);background:#0b0f14;border-radius:999px;font-size:10px;color:var(--muted);letter-spacing:.09em;text-transform:uppercase}
.dot{width:8px;height:8px;border-radius:50%;background:#50606b}
.link.on{color:var(--cyan);border-color:rgba(81,232,255,.25)}
.link.on .dot{background:var(--cyan);box-shadow:0 0 14px var(--cyan)}
.grid{display:grid;grid-template-columns:1.05fr .95fr;gap:12px}
.card{background:linear-gradient(180deg,rgba(255,255,255,.025),transparent),rgba(13,17,23,.94);
border:1px solid var(--line);border-radius:18px;padding:14px;box-shadow:0 14px 40px rgba(0,0,0,.25)}
.card h2{margin:0 0 10px;color:var(--muted);font-size:9px;letter-spacing:.16em;font-weight:900}
.hero{min-height:225px;display:flex;flex-direction:column;justify-content:space-between;position:relative;overflow:hidden}
.hero:after{content:"";position:absolute;width:155px;height:155px;right:-58px;top:-58px;border:1px solid rgba(81,232,255,.12);border-radius:50%;
box-shadow:0 0 0 25px rgba(81,232,255,.025),0 0 0 50px rgba(81,232,255,.012)}
.eyebrow{color:var(--cyan);font-size:9px;letter-spacing:.18em;font-weight:900}
.heroText{position:relative;z-index:1;margin:10px 0 4px;font-size:clamp(31px,6vw,58px);line-height:.94;font-weight:950;letter-spacing:-.045em;max-width:94%}
.heroSub{font-size:11px;color:var(--muted);max-width:90%}
.heroBottom{display:flex;justify-content:space-between;align-items:end;gap:12px;margin-top:16px}
.pill{display:inline-flex;align-items:center;gap:7px;border:1px solid var(--line);border-radius:999px;padding:7px 9px;color:var(--muted);font-size:9px;letter-spacing:.12em;text-transform:uppercase}
.metric{text-align:right}.metric .v{font-size:22px;font-weight:950}.metric .k{font-size:8px;color:var(--muted);letter-spacing:.12em}
.alertDumper{border-color:rgba(255,61,86,.5);box-shadow:0 0 36px rgba(255,61,86,.08) inset}
.alertTrack{border-color:rgba(255,184,63,.38);box-shadow:0 0 36px rgba(255,184,63,.07) inset}
.alertTrack.slope{border-color:rgba(81,232,255,.38)}
.alertGrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.tile{border:1px solid var(--line);background:#090d12;border-radius:14px;padding:12px;min-height:96px}
.tile.live.d{border-color:rgba(255,61,86,.5);box-shadow:inset 0 0 24px rgba(255,61,86,.05)}
.tile.live.t{border-color:rgba(255,184,63,.45);box-shadow:inset 0 0 24px rgba(255,184,63,.04)}
.tile .mini{font-size:8px;letter-spacing:.15em;color:var(--muted);text-transform:uppercase}
.tile .msg{margin-top:8px;font-weight:900;line-height:1.12}
.tile .age{margin-top:8px;color:var(--muted);font-size:9px}
.health{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.healthBox{padding:10px;border:1px solid var(--line);border-radius:12px;background:#090d12}
.healthBox .k{font-size:8px;letter-spacing:.1em;color:var(--muted);text-transform:uppercase}
.healthBox .v{margin-top:4px;font-weight:950;font-size:13px}
.ok{color:var(--green)}.warn{color:var(--amber)}.danger{color:var(--red)}
.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;width:min(360px,100%);margin:6px auto 0}
.control{height:65px;border:1px solid #273442;border-radius:15px;background:linear-gradient(180deg,#131c26,#0a1016);
color:#e6f5fb;font-size:23px;font-weight:950;display:flex;align-items:center;justify-content:center;touch-action:none;user-select:none;cursor:pointer}
.control:active,.control.pressed{border-color:rgba(81,232,255,.75);box-shadow:0 0 22px rgba(81,232,255,.12) inset;transform:scale(.987)}
.control.stop{border-color:rgba(255,61,86,.42);color:#ff7890}
.empty{visibility:hidden}
.driveState{margin:10px auto 0;width:min(360px,100%);padding:9px 12px;border:1px dashed #263644;border-radius:11px;background:#080d12;display:flex;justify-content:space-between}
.driveState .k{font-size:8px;letter-spacing:.15em;color:var(--muted)}.driveState strong{font-size:10px;color:var(--cyan);letter-spacing:.13em}
.sliderWrap{width:min(360px,100%);margin:12px auto 0}
.sliderWrap label{display:flex;justify-content:space-between;font-size:8px;color:var(--muted);letter-spacing:.12em;text-transform:uppercase;margin-bottom:5px}
.sliderWrap span:last-child{color:var(--cyan)}
input[type=range]{width:100%;accent-color:var(--cyan)}
.log{list-style:none;padding:0;margin:0;max-height:260px;overflow:auto}
.log li{display:grid;grid-template-columns:58px 1fr auto;gap:9px;padding:9px 0;border-bottom:1px solid rgba(255,255,255,.045);align-items:start}
.log li:last-child{border-bottom:0}.log small{color:var(--muted);font-size:8px}.log .msg{font-size:11px;font-weight:800}.log .src{margin-top:3px;color:#536876;font-size:8px;letter-spacing:.08em;text-transform:uppercase}.count{color:var(--cyan);font-size:9px}
.footer{text-align:center;margin-top:13px;color:#3d5564;font-size:8px;letter-spacing:.18em}
.overlay{position:fixed;inset:0;display:flex;align-items:center;justify-content:center;padding:16px;background:rgba(2,4,7,.76);backdrop-filter:blur(9px);z-index:100;opacity:0;pointer-events:none;transition:opacity .16s}
.overlay.show{opacity:1;pointer-events:auto}
.modal{width:min(690px,100%);padding:21px;border:1px solid rgba(81,232,255,.3);border-radius:22px;background:linear-gradient(180deg,#111923,#070b10);
box-shadow:0 0 70px rgba(81,232,255,.12),0 25px 90px rgba(0,0,0,.65);transform:scale(.96);transition:transform .16s}
.overlay.show .modal{transform:scale(1)}
.modal.d{border-color:rgba(255,61,86,.7);box-shadow:0 0 80px rgba(255,61,86,.16),0 25px 90px rgba(0,0,0,.65)}
.modal.t{border-color:rgba(255,184,63,.6)}
.modal .src{font-size:9px;letter-spacing:.18em;color:var(--cyan);font-weight:950;text-transform:uppercase}
.modal.d .src{color:#ff6f87}.modal.t .src{color:var(--amber)}
.modal h3{margin:15px 0 7px;font-size:clamp(31px,8vw,62px);line-height:.95;letter-spacing:-.04em}
.modal p{margin:0;color:var(--muted);font-size:11px}.ack{margin-top:17px;width:100%;height:46px;border-radius:11px;border:1px solid #2d3d4a;background:#0c141d;color:#dff7ff;font-weight:900;letter-spacing:.12em}
@media(max-width:760px){.grid{grid-template-columns:1fr}.hero{min-height:210px}}
@media(max-width:440px){.alertGrid{grid-template-columns:1fr}.title{font-size:18px}.controls{gap:7px}.control{height:62px}}
</style>
</head>
<body>
<main>
  <div class="top">
    <div class="brand">
      <div class="eye"><svg viewBox="0 0 100 60"><path d="M2 30Q50-10 98 30Q50 70 2 30Z" fill="none" stroke="#51e8ff" stroke-width="2.5"/><circle cx="50" cy="30" r="13" fill="#51e8ff" opacity=".13"/><circle cx="50" cy="30" r="7.5" fill="#51e8ff"/></svg></div>
      <div><div class="title">TRINETRA</div><div class="sub">CAR 1 // COMMAND &amp; AWARENESS CONSOLE</div></div>
    </div>
    <div id="link" class="link"><span class="dot"></span><span id="linkText">LINKING</span></div>
  </div>

  <div class="grid">
    <section id="heroCard" class="card hero">
      <div>
        <div id="heroTag" class="eyebrow">SYSTEM NOMINAL</div>
        <div id="heroText" class="heroText">ROAD CLEAR</div>
        <div id="heroSub" class="heroSub">TRINETRA is monitoring trackside and stationary-node alerts.</div>
      </div>
      <div class="heroBottom">
        <div class="pill"><span class="dot" style="background:var(--cyan);box-shadow:0 0 10px var(--cyan)"></span><span id="sourceBadge">ESP-NOW ONLINE</span></div>
        <div class="metric"><div id="eventAge" class="v">—</div><div class="k">LAST EVENT</div></div>
      </div>
    </section>

    <section class="card">
      <h2>DEVICE OVERVIEW // 01</h2>
      <div class="health">
        <div class="healthBox"><div class="k">Stationary</div><div id="stationaryLink" class="v warn">SEARCHING</div></div>
        <div class="healthBox"><div class="k">Track Node</div><div id="trackLink" class="v warn">SEARCHING</div></div>
        <div class="healthBox"><div class="k">Stationary RSSI</div><div id="stationaryRssi" class="v">—</div></div>
        <div class="healthBox"><div class="k">Track RSSI</div><div id="trackRssi" class="v">—</div></div>
      </div>
    </section>

    <section class="card">
      <h2>MANUAL DRIVE // HOLD TO MOVE</h2>
      <div class="controls">
        <button class="control empty" tabindex="-1">•</button>
        <button class="control" data-drive="forward">▲</button>
        <button class="control empty" tabindex="-1">•</button>
        <button class="control" data-drive="left">◀</button>
        <button class="control stop" data-drive="stop">■</button>
        <button class="control" data-drive="right">▶</button>
        <button class="control empty" tabindex="-1">•</button>
        <button class="control" data-drive="back">▼</button>
        <button class="control empty" tabindex="-1">•</button>
      </div>
      <div class="driveState"><span class="k">DRIVE STATE</span><strong id="driveState">STOPPED</strong></div>
      <div class="sliderWrap"><label><span>MOTOR POWER</span><span id="speedValue">180 / 255</span></label><input id="speed" type="range" min="80" max="255" value="180"></div>
    </section>

    <section class="card">
      <h2>ALERT MATRIX // 02</h2>
      <div class="alertGrid">
        <div id="trackTile" class="tile"><div class="mini">TRACK NODE 1</div><div id="trackText" class="msg">No alert yet</div><div id="trackAge" class="age">—</div></div>
        <div id="dumperTile" class="tile"><div class="mini">STATIONARY DUMPER</div><div id="dumperText" class="msg">No command yet</div><div id="dumperAge" class="age">—</div></div>
      </div>
    </section>

    <section class="card">
      <h2>EVENT STREAM // 03</h2>
      <ul id="log" class="log"></ul>
    </section>

    <section class="card">
      <h2>RADIO / SYSTEM</h2>
      <div class="health">
        <div class="healthBox"><div class="k">Access Point</div><div class="v">TRINETRA-CAR1</div></div>
        <div class="healthBox"><div class="k">Address</div><div class="v">192.168.4.1</div></div>
        <div class="healthBox"><div class="k">ESP-NOW</div><div class="v ok">CHANNEL 1</div></div>
        <div class="healthBox"><div class="k">Heartbeat</div><div class="v ok">1 Hz</div></div>
      </div>
    </section>
  </div>
  <div class="footer">STATIONARY DUMPER + TRACK NODE → MOVING DUMPER • NO INTERNET REQUIRED</div>
</main>

<div id="overlay" class="overlay">
  <div id="modal" class="modal">
    <div id="modalSource" class="src">ALERT</div>
    <h3 id="modalTitle">WARNING</h3>
    <p id="modalText">Incoming safety alert.</p>
    <button id="ack" class="ack">ACKNOWLEDGE</button>
  </div>
</div>

<script>
const $=id=>document.getElementById(id);
let popupTimer=0,driveTimer=0,activeDrive='stop',bootstrapped=false,lastSeq=0;

function ago(ms){
  if(ms<0)return'—'; const s=Math.floor(ms/1000);
  if(s<2)return'JUST NOW'; if(s<60)return s+'S AGO'; return Math.floor(s/60)+'M AGO';
}
function setLink(on){
  $('link').className='link'+(on?' on':'');
  $('linkText').textContent=on?'SYSTEM ONLINE':'LINK LOST';
}
function setDriveText(c){
  const m={forward:'FORWARD',back:'REVERSE',left:'PIVOT LEFT',right:'PIVOT RIGHT',stop:'STOPPED'};
  $('driveState').textContent=m[c]||'STOPPED';
}
async function drive(cmd){
  activeDrive=cmd; setDriveText(cmd);
  const sp=$('speed').value;
  try{await fetch('/drive?cmd='+encodeURIComponent(cmd)+'&speed='+sp,{cache:'no-store'});}
  catch(e){setLink(false);}
}
function startHold(cmd,e){
  if(e)e.preventDefault();
  if(cmd==='stop'){stopDrive();return;}
  activeDrive=cmd; drive(cmd);
  clearInterval(driveTimer);
  driveTimer=setInterval(()=>drive(cmd),180);
  document.querySelectorAll('.control').forEach(b=>b.classList.remove('pressed'));
  const b=document.querySelector('[data-drive="'+cmd+'"]'); if(b)b.classList.add('pressed');
}
function stopDrive(){
  clearInterval(driveTimer); driveTimer=0;
  document.querySelectorAll('.control').forEach(b=>b.classList.remove('pressed'));
  drive('stop');
}
function hidePopup(){clearTimeout(popupTimer);$('overlay').classList.remove('show');}
function showPopup(src,text,type){
  clearTimeout(popupTimer);
  $('modal').className='modal '+(src==='dumper'?'d':'t');
  $('modalSource').textContent=src==='dumper'?'STATIONARY DUMPER':'TRACK NODE 1';
  $('modalTitle').textContent=text;
  $('modalText').textContent=src==='dumper'
    ? 'Priority coordination command received from the stationary node.'
    : (type===2?'Slope warning received from Track Node 1.':'Curve warning received from Track Node 1.');
  $('overlay').classList.add('show');
  popupTimer=setTimeout(hidePopup,4200);
  try{if(navigator.vibrate)navigator.vibrate([90,50,90]);}catch(e){}
}
function render(d){
  const newest=d.log&&d.log.length?d.log[0]:null;
  if(!bootstrapped){lastSeq=newest?newest.seq:0;bootstrapped=true;}
  else if(newest&&newest.seq!==lastSeq){
    lastSeq=newest.seq;
    if(Number(newest.age)<2500)showPopup(newest.src===1?'dumper':'track',newest.text,newest.type);
  }

  const k=d.track||{}, du=d.dumper||{};
  const activeD=!!du.active, activeT=!!k.active;
  $('heroCard').className='card hero '+(activeD?'alertDumper':activeT?'alertTrack':'');
  if(activeD){
    $('heroTag').textContent='PRIORITY COMMAND';
    $('heroText').textContent=du.text||'MOVE LEFT';
    $('heroSub').textContent='Stationary dumper // '+ago(du.age);
    $('sourceBadge').textContent='DUMPER LINK ACTIVE';
    $('eventAge').textContent=ago(du.age);
  }else if(activeT){
    $('heroTag').textContent=k.type===2?'SLOPE WARNING':'CURVE WARNING';
    $('heroText').textContent=k.text||'TRACK ALERT';
    $('heroSub').textContent='Track Node 1 // '+ago(k.age);
    $('sourceBadge').textContent='TRACK LINK ACTIVE';
    $('eventAge').textContent=ago(k.age);
  }else{
    $('heroTag').textContent='SYSTEM NOMINAL';
    $('heroText').textContent='ROAD CLEAR';
    $('heroSub').textContent='TRINETRA is monitoring trackside and stationary-node alerts.';
    $('sourceBadge').textContent='ESP-NOW ONLINE';
    const aa=Math.min(k.age>=0?k.age:999999,du.age>=0?du.age:999999);
    $('eventAge').textContent=aa===999999?'—':ago(aa);
  }

  $('trackTile').className='tile'+(activeT?' live t':'');
  $('dumperTile').className='tile'+(activeD?' live d':'');
  $('trackText').textContent=k.age>=0?k.text:'No alert yet';
  $('trackAge').textContent=k.age>=0?ago(k.age):'—';
  $('dumperText').textContent=du.age>=0?du.text:'No command yet';
  $('dumperAge').textContent=du.age>=0?ago(du.age):'—';

  $('stationaryLink').textContent=d.stationaryLink?'CONNECTED':'SEARCHING / LOST';
  $('stationaryLink').className='v '+(d.stationaryLink?'ok':'warn');
  $('trackLink').textContent=d.trackLink?'CONNECTED':'SEARCHING / LOST';
  $('trackLink').className='v '+(d.trackLink?'ok':'warn');
  $('stationaryRssi').textContent=d.stationaryLink?(d.stationaryRssi+' dBm'):'—';
  $('trackRssi').textContent=d.trackLink?(d.trackRssi+' dBm'):'—';
  $('speedValue').textContent=d.speed+' / 255';
  setDriveText(d.drive==='F'?'forward':d.drive==='B'?'back':d.drive==='L'?'left':d.drive==='R'?'right':'stop');

  const ul=$('log');ul.textContent='';
  if(!d.log||!d.log.length){
    const e=document.createElement('li');e.innerHTML='<small>—</small><div><div class="msg">NO EVENTS RECEIVED</div><div class="src">WAITING</div></div><span class="count"></span>';ul.appendChild(e);return;
  }
  d.log.forEach(x=>{
    const li=document.createElement('li');
    const when=document.createElement('small');when.textContent=ago(x.age);
    const mid=document.createElement('div');
    const msg=document.createElement('div');msg.className='msg';msg.textContent=x.text;
    const src=document.createElement('div');src.className='src';src.textContent=x.src===1?'Stationary dumper':'Track node 1';
    mid.append(msg,src);
    const n=document.createElement('span');n.className='count';n.textContent=x.n>1?'x'+x.n:'';
    li.append(when,mid,n);ul.appendChild(li);
  });
}
async function tick(){
  try{const r=await fetch('/state',{cache:'no-store'});render(await r.json());setLink(true);}
  catch(e){setLink(false);}
}
document.querySelectorAll('.control').forEach(btn=>{
  const cmd=btn.dataset.drive;if(!cmd)return;
  btn.addEventListener('pointerdown',e=>startHold(cmd,e));
  btn.addEventListener('contextmenu',e=>e.preventDefault());
});
window.addEventListener('pointerup',stopDrive);
window.addEventListener('pointercancel',stopDrive);
window.addEventListener('blur',stopDrive);
document.addEventListener('keydown',e=>{
  if(e.repeat)return;const k=e.key.toLowerCase();
  const m={w:'forward',arrowup:'forward',s:'back',arrowdown:'back',a:'left',arrowleft:'left',d:'right',arrowright:'right'};
  if(k===' '||k==='escape'){e.preventDefault();stopDrive();return;}
  if(m[k]){e.preventDefault();startHold(m[k],e);}
});
document.addEventListener('keyup',e=>{
  const k=e.key.toLowerCase();
  if(['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright'].includes(k))stopDrive();
});
$('speed').addEventListener('input',()=>{
  $('speedValue').textContent=$('speed').value+' / 255';
  if(activeDrive!=='stop')drive(activeDrive);
});
$('ack').onclick=hidePopup;
$('overlay').addEventListener('click',e=>{if(e.target===$('overlay'))hidePopup();});
tick();setInterval(tick,250);
</script>
</body>
</html>
)rawliteral";

// ============================================================
// EVENT STORE HELPERS
// ============================================================
void recordEvent(uint8_t src, uint8_t type, const char* text)
{
  uint32_t now = millis();

  portENTER_CRITICAL(&mux);

  Event &latest = (src == SRC_DUMPER) ? lastDumper : lastTrack;
  latest.ms = now;
  latest.src = src;
  latest.type = type;
  latest.count = 1;
  strlcpy(latest.text, text, sizeof(latest.text));

  // The dumper repeats its command every second while the vehicle is close —
  // fold repeats into one log line instead of flooding the list.
  bool merged = false;
  if (logCount > 0) {
    Event &prev = logBuf[(logNext + LOG_SIZE - 1) % LOG_SIZE];
    if (prev.src == src && strcmp(prev.text, text) == 0 && (now - prev.ms) < 5000) {
      prev.ms = now;
      if (prev.count < 999) prev.count++;
      latest.seq = prev.seq;
      latest.count = prev.count;
      merged = true;
    }
  }

  if (!merged) {
    Event &e = logBuf[logNext];
    e.ms = now;
    e.src = src;
    e.type = type;
    e.count = 1;
    e.seq = ++eventSeq;
    latest.seq = e.seq;
    strlcpy(e.text, text, sizeof(e.text));
    logNext = (logNext + 1) % LOG_SIZE;
    if (logCount < LOG_SIZE) logCount++;
  }

  portEXIT_CRITICAL(&mux);
}

// Do not put Event in this function signature.
// Arduino's .ino auto-prototype generator can move the prototype before
// the Event declaration and produce: "Event does not name a type".
static void appendLatest(String &s, uint32_t ms, uint8_t type, const char* text, uint32_t seq, uint32_t now, uint32_t activeMs)
{
  bool seen = (ms != 0);
  uint32_t age = seen ? (now - ms) : 0;

  s += "{\"text\":\"";
  if (seen && text) s += text;
  s += "\",\"type\":";
  s += (int)type;
  s += ",\"age\":";
  if (seen) s += (unsigned long)age; else s += "-1";
  s += ",\"active\":";
  s += (seen && age < activeMs) ? "true" : "false";
  s += ",\"seq\":";
  s += (unsigned long)seq;
  s += "}";
}

// ============================================================
// WEB HANDLERS
// ============================================================
void handleRoot()
{
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

void handleState()
{
  uint32_t now = millis();

  Event d, t;
  Event lg[LOG_SIZE];
  uint8_t n;

  portENTER_CRITICAL(&mux);
  d = lastDumper;
  t = lastTrack;
  n = logCount;
  for (uint8_t i = 0; i < n; i++) {
    lg[i] = logBuf[(logNext + LOG_SIZE - 1 - i) % LOG_SIZE];   // newest first
  }
  portEXIT_CRITICAL(&mux);

  String s;
  s.reserve(900);
  s += "{\"dumper\":";
  appendLatest(s, d.ms, d.type, d.text, d.seq, now, DUMPER_ACTIVE_MS);
  s += ",\"track\":";
  appendLatest(s, t.ms, t.type, t.text, t.seq, now, TRACK_ACTIVE_MS);
  s += ",\"stationaryLink\":";
  s += stationaryLinked() ? "true" : "false";
  s += ",\"trackLink\":";
  s += trackLinked() ? "true" : "false";
  s += ",\"stationaryRssi\":";
  s += stationaryLinked() ? String(stationaryRssi) : String(0);
  s += ",\"trackRssi\":";
  s += trackLinked() ? String(trackRssi) : String(0);
  s += ",\"drive\":\"";
  char driveChar = 'S';
  switch(currentDrive){
    case DRIVE_FORWARD: driveChar='F'; break;
    case DRIVE_BACK: driveChar='B'; break;
    case DRIVE_LEFT: driveChar='L'; break;
    case DRIVE_RIGHT: driveChar='R'; break;
    default: driveChar='S'; break;
  }
  s += driveChar;
  s += "\",\"speed\":";
  s += (int)currentSpeed;
  s += ",\"log\":[";
  for (uint8_t i = 0; i < n; i++) {
    if (i) s += ",";
    s += "{\"src\":";
    s += (int)lg[i].src;
    s += ",\"text\":\"";
    s += lg[i].text;
    s += "\",\"age\":";
    s += (unsigned long)(now - lg[i].ms);
    s += ",\"n\":";
    s += (int)lg[i].count;
    s += ",\"seq\":";
    s += (unsigned long)lg[i].seq;
    s += "}";
  }
  s += "]}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

// ============================================================
// WEB DRIVE HANDLER
// ============================================================
void handleDrive()
{
  if (!server.hasArg("cmd")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing cmd\"}");
    return;
  }

  String cmd = server.arg("cmd");
  if (cmd.length() > 16) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"bad cmd\"}");
    return;
  }
  cmd.toLowerCase();

  int requestedSpeed = currentSpeed;
  if (server.hasArg("speed")) requestedSpeed = constrain(server.arg("speed").toInt(), 80, 255);
  currentSpeed = (uint8_t)requestedSpeed;

  DriveCommand next = DRIVE_STOP;
  if      (cmd == "forward") next = DRIVE_FORWARD;
  else if (cmd == "back")    next = DRIVE_BACK;
  else if (cmd == "left")    next = DRIVE_LEFT;
  else if (cmd == "right")   next = DRIVE_RIGHT;
  else if (cmd == "stop")    next = DRIVE_STOP;
  else {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"bad cmd\"}");
    return;
  }

  setDrive(next, currentSpeed);

  const char* name = "STOP";
  if (next == DRIVE_FORWARD) name = "FORWARD";
  else if (next == DRIVE_BACK) name = "REVERSE";
  else if (next == DRIVE_LEFT) name = "PIVOT LEFT";
  else if (next == DRIVE_RIGHT) name = "PIVOT RIGHT";

  String out = "{\"ok\":true,\"drive\":\"";
  out += name;
  out += "\",\"speed\":";
  out += (int)currentSpeed;
  out += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

// ============================================================
// MOTOR CONTROL
// ============================================================
void applyMotorSide(int en, int in1, int in2, int direction, uint8_t pwm, bool reversed)
{
  if (direction == 0 || pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(en, 0);
    return;
  }

  if (reversed) direction = -direction;

  digitalWrite(in1, direction > 0 ? HIGH : LOW);
  digitalWrite(in2, direction > 0 ? LOW : HIGH);
  ledcWrite(en, pwm);
}

void setDrive(DriveCommand cmd, uint8_t pwm)
{
  int leftDir = 0;
  int rightDir = 0;

  switch (cmd) {
    case DRIVE_FORWARD: leftDir = +1; rightDir = +1; break;
    case DRIVE_BACK:    leftDir = -1; rightDir = -1; break;
    case DRIVE_LEFT:    leftDir = -1; rightDir = +1; break;
    case DRIVE_RIGHT:   leftDir = +1; rightDir = -1; break;
    case DRIVE_STOP:
    default:            leftDir = 0;  rightDir = 0;  break;
  }

  applyMotorSide(LEFT_EN, LEFT_IN1, LEFT_IN2, leftDir, pwm, LEFT_MOTOR_REVERSED);
  applyMotorSide(RIGHT_EN, RIGHT_IN1, RIGHT_IN2, rightDir, pwm, RIGHT_MOTOR_REVERSED);
  currentDrive = cmd;
  lastDriveCommandMs = millis();
}

void stopMotors()
{
  applyMotorSide(LEFT_EN, LEFT_IN1, LEFT_IN2, 0, 0, LEFT_MOTOR_REVERSED);
  applyMotorSide(RIGHT_EN, RIGHT_IN1, RIGHT_IN2, 0, 0, RIGHT_MOTOR_REVERSED);
  currentDrive = DRIVE_STOP;
}

// ============================================================
// ESP-NOW
// ============================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (!info || !data) return;
  if (len != (int)sizeof(AlertPacket)) return;

  AlertPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  pkt.message[sizeof(pkt.message) - 1] = '\0';

  // keep the JSON valid: printable ASCII only, no quotes or backslashes
  for (char *p = pkt.message; *p; ++p) {
    if (*p < 32 || *p > 126 || *p == '"' || *p == '\\') *p = ' ';
  }

  switch (pkt.messageType) {
    case MSG_CURVE:
    case MSG_SLOPE:
      recordEvent(SRC_TRACK, pkt.messageType, pkt.message);
      Serial.print("[TRACK NODE] ");
      Serial.println(pkt.message);
      break;

    case MSG_DUMPER_CMD:
      // The stationary LiDAR/TFT node sends this directly to the car.
      // Do not involve the Track node here.
      recordEvent(SRC_DUMPER, pkt.messageType, pkt.message);

      Serial.print("[STATIONARY DUMPER] COMMAND RECEIVED: ");
      Serial.println(pkt.message);

      if (strcmp(pkt.message, MOVE_LEFT_COMMAND) == 0) {
        Serial.println("[CAR] MOVE LEFT received successfully.");
        // Intentionally do NOT drive the motors automatically.
        // The command is displayed on the dashboard so the existing
        // manual-drive system remains unchanged.
      }
      break;

    case MSG_NODE_HEARTBEAT:
      if (strcmp(pkt.message, "STATIONARY_ALIVE") == 0) {
        lastStationaryHeartbeat = millis();
        if (info->rx_ctrl) stationaryRssi = info->rx_ctrl->rssi;
      } else if (strcmp(pkt.message, "TRACK_ALIVE") == 0) {
        lastTrackHeartbeat = millis();
        if (info->rx_ctrl) trackRssi = info->rx_ctrl->rssi;
      }
      break;

    default:
      break;   // car heartbeat / unknown types: ignore
  }
}

void sendHeartbeat()
{
  AlertPacket pkt = {};
  pkt.messageType = MSG_CAR_HEARTBEAT;
  strlcpy(pkt.message, "CAR1", sizeof(pkt.message));
  esp_now_send(broadcastAddr, (uint8_t *)&pkt, sizeof(pkt));
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== TRINETRA CAR 1 ===");

  // ---- Motor driver ----
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  ledcAttach(LEFT_EN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttach(RIGHT_EN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  stopMotors();
  lastDriveCommandMs = millis();

  // AP + STA: the AP hosts the web page, the STA side receives ESP-NOW.
  // Both sit on WIFI_CHANNEL, so ESP-NOW stays on channel 1.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  if (!WiFi.softAP(AP_SSID, AP_PASS, WIFI_CHANNEL)) {
    Serial.println("Access point failed to start");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, restarting...");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t bc = {};
  memcpy(bc.peer_addr, broadcastAddr, 6);
  bc.channel = WIFI_CHANNEL;
  bc.encrypt = false;
  esp_err_t peerResult = esp_now_add_peer(&bc);
  if (peerResult != ESP_OK && peerResult != ESP_ERR_ESPNOW_EXIST) {
    Serial.print("Broadcast peer add failed: ");
    Serial.println((int)peerResult);
  }

  server.on("/", handleRoot);
  server.on("/state", handleState);
  server.on("/drive", handleDrive);
  server.begin();

  Serial.print("Car MAC (must be 00:70:07:E1:BC:EC): ");
  Serial.println(WiFi.macAddress());
  Serial.print("Wi-Fi: ");
  Serial.print(AP_SSID);
  Serial.print("   Page: http://");
  Serial.println(WiFi.softAPIP());
}

void loop()
{
  server.handleClient();

  unsigned long now = millis();
  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    sendHeartbeat();
  }

  // Dead-man safety: if the browser stops refreshing while a direction is held, stop.
  if (currentDrive != DRIVE_STOP && (now - lastDriveCommandMs) > DRIVE_FAILSAFE_MS) {
    stopMotors();
  }

  delay(2);
}
