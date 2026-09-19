//moving car controller with alerts


#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include "esp_wifi.h"

// ==========================================
// WIFI
// ==========================================

const char* ssid = "Robot-ESP32";
const char* password = "12345678";

#define WIFI_CHANNEL 1

WebServer server(80);


// ==========================================
// L298N PINS
// ==========================================

// LEFT SIDE
#define ENA 25
#define IN1 26
#define IN2 27

// RIGHT SIDE
#define ENB 14
#define IN3 18
#define IN4 19


// ==========================================
// TIMING SETTINGS
// ==========================================

#define DESTINATION_TIMEOUT_MS 5000
#define FAILSAFE_TIMEOUT_MS    600
#define MSG_REPEAT_GAP_MS      4000


// ==========================================
// MOTOR FUNCTIONS
// ==========================================

void leftForward() {
  digitalWrite(ENA, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void leftBackward() {
  digitalWrite(ENA, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void leftStop() {
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void rightForward() {
  digitalWrite(ENB, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void rightBackward() {
  digitalWrite(ENB, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void rightStop() {
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  leftStop();
  rightStop();
}


// ==========================================
// ROBOT MOVEMENTS
// ==========================================

void forward() {
  leftForward();
  rightForward();
}

void backward() {
  leftBackward();
  rightBackward();
}

void turnLeft() {
  leftBackward();
  rightForward();
}

void turnRight() {
  leftForward();
  rightBackward();
}


// ==========================================
// DRIVE COMMAND TRACKING
// ==========================================

unsigned long lastMoveCmd = 0;

bool hasMoved = false;
bool motorsRunning = false;

void markMoveCommand() {
  lastMoveCmd = millis();
  hasMoved = true;
  motorsRunning = true;
}


// ==========================================
// ESP-NOW TRACK NODES
// ==========================================

#define RSSI_REF_AT_1M -40.0f
#define PATH_LOSS_N    2.5f


// ==========================================
// MESSAGE STRUCTURE
// MUST MATCH NODE CODE
// ==========================================

typedef struct struct_message {
  char nodeName[16];
  char message[32];
} struct_message;

struct_message outgoingPing;


// ==========================================
// KNOWN NODES
// ==========================================

struct KnownPeer {
  const char* name;
  uint8_t mac[6];
};


// IMPORTANT:
// Names shown on dashboard are now NODE 1 and NODE 2.
// MAC addresses remain unchanged.

KnownPeer knownPeers[] = {

  {
    "NODE 1",
    {0x00, 0x70, 0x07, 0xE3, 0x3B, 0x80}
  },

  {
    "NODE 2",
    {0xB0, 0xCB, 0xD8, 0xE8, 0x76, 0x04}
  }

};

const int NUM_KNOWN_PEERS = 2;


// ==========================================
// NODE STATE
// ==========================================

struct NodeState {

  int rssi;

  float distance;
  float prevDistance;
  float eta;

  unsigned long lastSeen;
  unsigned long prevSeen;

  bool everSeen;

  // Last received message
  char msg[32];

  unsigned long msgTime;

  // Changes whenever a new popup should appear
  uint32_t msgId;
};


NodeState nodeStates[NUM_KNOWN_PEERS];


// ==========================================
// FIND NODE BY MAC
// ==========================================

int findPeerIndexByMac(const uint8_t* mac) {

  for (int i = 0; i < NUM_KNOWN_PEERS; i++) {

    if (memcmp(knownPeers[i].mac, mac, 6) == 0) {
      return i;
    }

  }

  return -1;
}


// ==========================================
// FIND NODE BY NAME
// ==========================================

int findPeerIndexByName(const char* name) {

  for (int i = 0; i < NUM_KNOWN_PEERS; i++) {

    if (strcmp(knownPeers[i].name, name) == 0) {
      return i;
    }

  }

  return -1;
}


// ==========================================
// UPDATE NODE RSSI / DISTANCE
// ==========================================

void updateNodeState(int idx, int rssi) {

  NodeState &n = nodeStates[idx];

  float distance =
    pow(
      10.0,
      (RSSI_REF_AT_1M - rssi) /
      (10.0 * PATH_LOSS_N)
    );

  unsigned long now = millis();


  if (n.everSeen) {

    float dt =
      (now - n.prevSeen) / 1000.0f;

    if (dt > 0.05f) {

      float speed =
        (n.prevDistance - distance) / dt;

      // Positive = approaching
      n.eta =
        (speed > 0.01f)
        ? (distance / speed)
        : -1.0f;
    }

  } else {

    n.eta = -1.0f;

  }


  n.rssi = rssi;

  n.prevDistance = distance;
  n.distance = distance;

  n.prevSeen = now;
  n.lastSeen = now;

  n.everSeen = true;
}


// ==========================================
// UPDATE NODE MESSAGE
// ==========================================

void updateNodeMessage(
  int idx,
  const char* text
) {

  NodeState &n = nodeStates[idx];

  unsigned long now = millis();


  // New message if:
  // 1. Text changed
  // OR
  // 2. Same message returned after 4 seconds

  bool changed =
    (strncmp(n.msg, text, sizeof(n.msg)) != 0);


  if (
    changed ||
    (now - n.msgTime) > MSG_REPEAT_GAP_MS
  ) {

    n.msgId++;

  }


  strncpy(
    n.msg,
    text,
    sizeof(n.msg) - 1
  );

  n.msg[
    sizeof(n.msg) - 1
  ] = '\0';

  n.msgTime = now;
}


// ==========================================
// ESP-NOW RECEIVE CALLBACK
// ==========================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {

  struct_message incoming;

  memset(
    &incoming,
    0,
    sizeof(incoming)
  );


  memcpy(
    &incoming,
    incomingData,
    min(
      (size_t)len,
      sizeof(incoming)
    )
  );


  incoming.nodeName[
    sizeof(incoming.nodeName) - 1
  ] = '\0';


  incoming.message[
    sizeof(incoming.message) - 1
  ] = '\0';


  int idx =
    findPeerIndexByMac(info->src_addr);


  // Fallback
  if (idx == -1) {

    idx =
      findPeerIndexByName(
        incoming.nodeName
      );

  }


  if (idx != -1) {

    int rssi =
      info->rx_ctrl->rssi;


    updateNodeState(
      idx,
      rssi
    );


    // Receive message
    if (incoming.message[0] != '\0') {

      updateNodeMessage(
        idx,
        incoming.message
      );

    }


    // Serial monitor
    Serial.print("[RX] ");

    Serial.print(
      knownPeers[idx].name
    );

    Serial.print(" | RSSI=");

    Serial.print(rssi);

    Serial.print(" | DIST=");

    Serial.print(
      nodeStates[idx].distance,
      2
    );

    Serial.print(" m");


    if (incoming.message[0] != '\0') {

      Serial.print(" | MSG=");

      Serial.print(
        incoming.message
      );

    }

    Serial.println();

  }

}


// ==========================================
// ESP-NOW SEND CALLBACK
// ==========================================

void OnDataSent(
  const wifi_tx_info_t *tx_info,
  esp_now_send_status_t status
) {

  Serial.print(
    "esp_now_send status: "
  );

  Serial.println(
    status == ESP_NOW_SEND_SUCCESS
    ? "ACCEPTED"
    : "ERROR"
  );

}


// ==========================================
// WEB PAGE
// ==========================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html lang="en">

<head>

<meta charset="utf-8">

<meta
  name="viewport"
  content="width=device-width,
           initial-scale=1,
           user-scalable=no"
>

<meta
  name="theme-color"
  content="#0c0807"
>

<title>
Bailadila Haul Console
</title>


<style>


/* ==========================================
   VARIABLES
   ========================================== */

:root {

  --bg1: #0c0807;
  --bg2: #1d100b;

  --card: rgba(255,255,255,0.05);

  --border:
    rgba(217,84,30,0.28);

  --text: #f3ebe6;

  --muted: #a3928a;

  --ore: #d9541e;

  --ore2: #ff7a3d;

  --amber: #ffb400;

  --ok: #3ecf6e;

  --danger: #ff3b30;

}


/* ==========================================
   GLOBAL
   ========================================== */

* {
  box-sizing: border-box;
}


html,
body {

  margin: 0;

  min-height: 100%;

  font-family:
    "Bahnschrift",
    "Segoe UI",
    system-ui,
    sans-serif;

  color: var(--text);

  background:

    radial-gradient(
      ellipse at 15% -10%,
      #3a1a0e 0%,
      transparent 55%
    ),

    linear-gradient(
      180deg,
      var(--bg2),
      var(--bg1) 70%
    );

  background-attachment: fixed;

  user-select: none;

  -webkit-user-select: none;

  touch-action: manipulation;

}


/* ==========================================
   TERRAIN
   ========================================== */

#terrain {

  position: fixed;

  left: 0;
  right: 0;
  bottom: 0;

  height: 40vh;

  z-index: 0;

  pointer-events: none;

}


.wrap {

  position: relative;

  z-index: 1;

  max-width: 1000px;

  margin: 0 auto;

  padding:
    18px
    16px
    40px;

}


/* ==========================================
   HEADER
   ========================================== */

header {

  display: flex;

  align-items: center;

  justify-content:
    space-between;

  flex-wrap: wrap;

  gap: 12px;

}


.brand {

  display: flex;

  align-items: center;

  gap: 14px;

}


.logo {

  width: 54px;

  height: 54px;

  border-radius: 14px;

  background:
    linear-gradient(
      135deg,
      #3a1a0e,
      #1b100b
    );

  border:
    1px solid var(--border);

  display: grid;

  place-items: center;

  box-shadow:
    0 6px 22px
    rgba(217,84,30,0.35);

}


.logo svg {

  width: 38px;

  height: 38px;

}


.over {

  font-size: 11px;

  letter-spacing: 2.5px;

  color: var(--ore2);

  font-weight: 700;

}


h1 {

  margin: 2px 0 0;

  font-size: 22px;

  letter-spacing: 1.5px;

  text-transform: uppercase;

}


.sub {

  font-size: 12px;

  color: var(--muted);

  margin-top: 2px;

}


.pills {

  display: flex;

  gap: 8px;

  flex-wrap: wrap;

}


.pill {

  display: flex;

  align-items: center;

  gap: 8px;

  padding:
    7px
    14px;

  border-radius: 999px;

  background: var(--card);

  border:
    1px solid var(--border);

  color: var(--text);

  font-family: inherit;

  font-size: 13px;

}


button.pill {

  cursor: pointer;

}


.dot {

  width: 9px;

  height: 9px;

  border-radius: 50%;

  background: var(--muted);

  flex: none;

}


.dot.on {

  background: var(--ok);

  box-shadow:
    0 0 10px var(--ok);

}


.dot.off {

  background: var(--danger);

  box-shadow:
    0 0 10px var(--danger);

}


.hazard {

  height: 7px;

  margin: 16px 0 0;

  border-radius: 4px;

  background:
    repeating-linear-gradient(
      -45deg,
      var(--amber) 0 12px,
      #141010 12px 24px
    );

  opacity: .9;

}


/* ==========================================
   ROAD STATUS
   ========================================== */

.alert {

  display: flex;

  align-items: center;

  flex-wrap: wrap;

  gap: 14px;

  padding:
    14px
    18px;

  border-radius: 18px;

  margin: 16px 0;

  border: 1px solid;

  transition:
    background .25s,
    border-color .25s;

}


.alert.ok {

  background:
    rgba(62,207,110,.10);

  border-color:
    rgba(62,207,110,.45);

}


.alert.warn {

  background:
    rgba(255,180,0,.14);

  border-color:
    rgba(255,180,0,.65);

}


.alert.danger {

  background:
    rgba(255,59,48,.16);

  border-color:
    rgba(255,59,48,.75);

  animation:
    pulse 1s infinite;

}


@keyframes pulse {

  50% {

    box-shadow:
      0 0 0 7px
      rgba(255,59,48,.18);

  }

}


.a-ico {

  font-size: 30px;

}


.a-lbl {

  font-size: 11px;

  letter-spacing: 2px;

  color: var(--muted);

}


.a-txt {

  font-size: 20px;

  font-weight: 800;

  letter-spacing: 1px;

}


.a-src {

  margin-left: auto;

  font-size: 13px;

  color: var(--muted);

  text-align: right;

}


/* ==========================================
   GRID
   ========================================== */

.grid {

  display: grid;

  grid-template-columns:
    1fr 1fr;

  gap: 16px;

}


@media (max-width: 760px) {

  .grid {

    grid-template-columns:
      1fr;

  }

}


/* ==========================================
   CARDS
   ========================================== */

.card {

  position: relative;

  overflow: hidden;

  background: var(--card);

  border:
    1px solid var(--border);

  border-radius: 20px;

  padding:
    22px
    20px
    20px;

  backdrop-filter:
    blur(10px);

  -webkit-backdrop-filter:
    blur(10px);

}


.card::before {

  content: "";

  position: absolute;

  left: 0;
  top: 0;
  right: 0;

  height: 3px;

  background:
    linear-gradient(
      90deg,
      var(--ore),
      var(--amber)
    );

}


.card h2 {

  margin:
    0 0 14px;

  font-size: 13px;

  font-weight: 700;

  text-transform: uppercase;

  letter-spacing: 2px;

  color: var(--muted);

}


/* ==========================================
   D-PAD
   ========================================== */

.dpad {

  display: grid;

  grid-template-columns:
    repeat(3, 84px);

  grid-template-rows:
    repeat(3, 84px);

  gap: 10px;

  justify-content: center;

  margin:
    6px 0 14px;

}


.key {

  border:
    1px solid var(--border);

  background:
    rgba(255,255,255,0.06);

  color: var(--text);

  border-radius: 18px;

  font-size: 26px;

  font-family: inherit;

  cursor: pointer;

  display: flex;

  flex-direction: column;

  align-items: center;

  justify-content: center;

  line-height: 1.1;

  transition:
    transform .06s,
    background .12s,
    box-shadow .12s;

  touch-action: none;

}


.key small {

  font-size: 11px;

  color: var(--muted);

  margin-top: 3px;

}


.key.active {

  background:
    linear-gradient(
      135deg,
      var(--ore),
      var(--ore2)
    );

  box-shadow:
    0 8px 24px
    rgba(217,84,30,0.55);

  transform:
    scale(0.94);

  border-color: transparent;

}


.key.active small {

  color: #fff;

}


.key.stop {

  background:
    rgba(255,59,48,0.18);

  border-color:
    rgba(255,59,48,0.55);

  font-size: 15px;

  font-weight: 800;

  letter-spacing: 1px;

}


.key.stop:active {

  background:
    var(--danger);

}


.hint {

  text-align: center;

  font-size: 12px;

  color: var(--muted);

  line-height: 1.9;

}


.hint kbd {

  background:
    rgba(255,255,255,0.1);

  border:
    1px solid var(--border);

  border-radius: 6px;

  padding:
    1px 7px;

  font-size: 11px;

  margin: 0 1px;

}


.state {

  margin-top: 14px;

  text-align: center;

  font-size: 14px;

  color: var(--muted);

}


.state b {

  color: var(--amber);

  letter-spacing: 2px;

}


/* ==========================================
   NODE CARDS
   ========================================== */

.node {

  border:
    1px solid var(--border);

  background:
    rgba(255,255,255,0.035);

  border-radius: 16px;

  padding:
    14px
    16px;

  margin-bottom: 12px;

}


.node:last-child {

  margin-bottom: 0;

}


.node-top {

  display: flex;

  align-items: center;

  justify-content:
    space-between;

  margin-bottom: 10px;

}


.node-name {

  font-weight: 700;

  font-size: 16px;

  display: flex;

  align-items: center;

  gap: 8px;

  letter-spacing: .5px;

}


.age {

  font-size: 12px;

  color: var(--muted);

}


.stats {

  display: grid;

  grid-template-columns:
    repeat(3, 1fr);

  gap: 8px;

  text-align: center;

}


.stat .v {

  font-size: 18px;

  font-weight: 700;

}


.stat .l {

  font-size: 11px;

  color: var(--muted);

  text-transform: uppercase;

  letter-spacing: 1px;

}


.prox {

  margin-top: 12px;

}


.bar {

  height: 8px;

  border-radius: 99px;

  background:
    rgba(255,255,255,.08);

  overflow: hidden;

}


.fill {

  height: 100%;

  width: 0;

  border-radius: 99px;

  background: var(--ok);

  transition:
    width .4s,
    background .3s;

}


.fill.z1 {

  background:
    var(--amber);

}


.fill.z2 {

  background:
    var(--danger);

}


.zrow {

  display: flex;

  justify-content:
    space-between;

  font-size: 11px;

  letter-spacing: 1.5px;

  color: var(--muted);

  margin-top: 5px;

}


.zrow .z0 {

  color: var(--ok);

}


.zrow .z1 {

  color: var(--amber);

}


.zrow .z2 {

  color: var(--danger);

}


.lastmsg {

  margin-top: 10px;

  padding:
    8px
    10px;

  border-radius: 10px;

  background:
    rgba(255,255,255,0.05);

  font-size: 13px;

  color: var(--muted);

}


.lastmsg b {

  color: var(--text);

  letter-spacing: .5px;

}


.lastmsg b.l1 {

  color: var(--amber);

}


.lastmsg b.l2 {

  color: var(--danger);

}


.empty {

  color: var(--muted);

  font-size: 14px;

}


/* ==========================================
   FOOTER
   ========================================== */

.foot {

  margin-top: 22px;

  text-align: center;

  font-size: 12px;

  color: var(--muted);

  letter-spacing: 1.5px;

}


.foot b {

  color: var(--ore2);

}


/* ==========================================
   MESSAGE POPUPS
   ========================================== */

#toasts {

  position: fixed;

  top: 20px;

  left: 50%;

  transform:
    translateX(-50%);

  width:
    min(420px, 90vw);

  z-index: 50;

  display: flex;

  flex-direction: column;

  gap: 10px;

  pointer-events: none;

}


.toast {

  display: flex;

  gap: 13px;

  align-items: center;

  padding:
    14px
    18px;

  border-radius: 16px;

  background:
    rgba(24,14,10,0.97);

  border:
    1px solid var(--border);

  border-left:
    6px solid var(--ore2);

  box-shadow:
    0 12px 32px
    rgba(0,0,0,0.55);

  animation:
    slideIn .28s ease-out;

}


.toast.warn {

  border-left-color:
    var(--amber);

}


.toast.danger {

  border-left-color:
    var(--danger);

}


.toast.ok {

  border-left-color:
    var(--ok);

}


.toast .ico {

  font-size: 28px;

  flex: none;

}


.toast .from {

  font-size: 11px;

  color: var(--muted);

  text-transform: uppercase;

  letter-spacing: 1.5px;

  margin-bottom: 2px;

}


.toast .txt {

  font-size: 19px;

  font-weight: 800;

  letter-spacing: .5px;

}


.toast.hide {

  animation:
    slideOut .3s ease-in forwards;

}


@keyframes slideIn {

  from {

    opacity: 0;

    transform:
      translateY(-20px)
      scale(.96);

  }

  to {

    opacity: 1;

    transform:
      none;

  }

}


@keyframes slideOut {

  to {

    opacity: 0;

    transform:
      translateY(-20px)
      scale(.96);

  }

}


/* ==========================================
   DESTINATION REACHED
   ========================================== */

#reached {

  position: fixed;

  inset: 0;

  background:
    rgba(8,4,3,0.80);

  backdrop-filter:
    blur(6px);

  -webkit-backdrop-filter:
    blur(6px);

  display: none;

  align-items: center;

  justify-content: center;

  z-index: 100;

}


#reached.show {

  display: flex;

}


.reached-card {

  text-align: center;

  padding:
    34px
    40px
    30px;

  border-radius: 26px;

  background:
    linear-gradient(
      160deg,
      rgba(255,180,0,0.18),
      rgba(24,14,10,0.97)
    );

  border:
    1px solid
    rgba(255,180,0,0.6);

  box-shadow:
    0 20px 60px
    rgba(255,140,0,0.25);

  animation:
    pop .35s ease-out;

  max-width: 92vw;

}


.reached-card .big {

  font-size: 60px;

}


.reached-card h3 {

  margin:
    8px 0 4px;

  font-size: 26px;

  letter-spacing: 2px;

  text-transform: uppercase;

}


.reached-card p {

  margin: 0;

  color: var(--muted);

  font-size: 14px;

}


.reached-card .hz {

  height: 6px;

  border-radius: 3px;

  margin-top: 16px;

  background:
    repeating-linear-gradient(
      -45deg,
      var(--amber) 0 10px,
      #141010 10px 20px
    );

}


@keyframes pop {

  from {

    opacity: 0;

    transform:
      scale(.85);

  }

  to {

    opacity: 1;

    transform:
      none;

  }

}

</style>

</head>


<body>


<!-- ======================================
     TERRAIN
     ====================================== -->

<svg
  id="terrain"
  viewBox="0 0 1200 220"
  preserveAspectRatio="xMidYMax slice"
  xmlns="http://www.w3.org/2000/svg"
>

  <path
    d="
      M0 140
      L120 90
      L220 120
      L340 60
      L470 110
      L600 70
      L740 120
      L880 80
      L1020 115
      L1200 75
      L1200 220
      L0 220
      Z
    "
    fill="rgba(140,50,20,0.22)"
  />

  <path
    d="
      M0 220
      L0 165
      L90 165
      L90 176
      L200 176
      L200 188
      L330 188
      L330 199
      L470 199
      L470 210
      L730 210
      L730 199
      L870 199
      L870 188
      L1000 188
      L1000 176
      L1110 176
      L1110 165
      L1200 165
      L1200 220
      Z
    "
    fill="rgba(217,84,30,0.20)"
  />

  <path
    d="
      M0 220
      L0 190
      L140 190
      L140 200
      L300 200
      L300 210
      L900 210
      L900 200
      L1060 200
      L1060 190
      L1200 190
      L1200 220
      Z
    "
    fill="rgba(255,122,61,0.10)"
  />

</svg>


<!-- ======================================
     POPUPS
     ====================================== -->

<div id="toasts"></div>


<!-- ======================================
     DESTINATION POPUP
     ====================================== -->

<div id="reached">

  <div class="reached-card">

    <div class="big">
      🏁
    </div>

    <h3>
      Destination Reached
    </h3>

    <p>
      Dumper halted · no command
      for 5 seconds
      <br>
      Press any drive key to continue
    </p>

    <div class="hz"></div>

  </div>

</div>


<div class="wrap">


<!-- ======================================
     HEADER
     ====================================== -->

<header>

  <div class="brand">

    <div class="logo">

      <svg
        viewBox="0 0 64 64"
        xmlns="http://www.w3.org/2000/svg"
      >

        <path
          d="M5 42 L9 21 L40 21 L46 42 Z"
          fill="#ffb400"
        />

        <path
          d="M9 21 L40 21"
          stroke="#141010"
          stroke-width="3"
        />

        <rect
          x="42"
          y="27"
          width="17"
          height="15"
          rx="3"
          fill="#d9541e"
        />

        <rect
          x="47"
          y="30"
          width="8"
          height="6"
          rx="1"
          fill="#141010"
          opacity=".7"
        />

        <circle
          cx="19"
          cy="47"
          r="8"
          fill="#1b1310"
          stroke="#ffb400"
          stroke-width="3"
        />

        <circle
          cx="49"
          cy="47"
          r="8"
          fill="#1b1310"
          stroke="#ffb400"
          stroke-width="3"
        />

      </svg>

    </div>


    <div>

      <div class="over">
        NMDC · BAILADILA IRON ORE MINE
      </div>

      <h1>
        Haul Road Safety Console
      </h1>

      <div class="sub">
        Kirandul · Bailadila Range · Dumper 01
      </div>

    </div>

  </div>


  <div class="pills">

    <div class="pill">

      🕒

      <span id="clock">
        --:--:--
      </span>

    </div>


    <button
      class="pill"
      id="soundBtn"
    >
      🔔 Sound on
    </button>


    <div class="pill">

      <span
        class="dot"
        id="linkDot"
      ></span>

      <span id="linkTxt">
        Connecting…
      </span>

    </div>

  </div>

</header>


<div class="hazard"></div>


<!-- ======================================
     ROAD STATUS
     ====================================== -->

<div
  class="alert ok"
  id="alert"
>

  <div
    class="a-ico"
    id="aIco"
  >
    ✅
  </div>


  <div>

    <div class="a-lbl">
      ROAD STATUS
    </div>

    <div
      class="a-txt"
      id="aTxt"
    >
      ROAD CLEAR
    </div>

  </div>


  <div
    class="a-src"
    id="aSrc"
  >
    No hazards reported
  </div>

</div>


<!-- ======================================
     MAIN GRID
     ====================================== -->

<div class="grid">


<!-- ======================================
     DUMPER CONTROL
     ====================================== -->

<section class="card">

  <h2>
    Dumper Control
  </h2>


  <div class="dpad">

    <span></span>

    <button
      class="key"
      id="k-forward"
      data-cmd="forward"
    >
      ▲
      <small>
        W / ↑
      </small>
    </button>

    <span></span>


    <button
      class="key"
      id="k-left"
      data-cmd="left"
    >
      ◀
      <small>
        A / ←
      </small>
    </button>


    <button
      class="key stop"
      id="k-stop"
      data-cmd="stop"
    >
      STOP
      <small>
        Space
      </small>
    </button>


    <button
      class="key"
      id="k-right"
      data-cmd="right"
    >
      ▶
      <small>
        D / →
      </small>
    </button>


    <span></span>


    <button
      class="key"
      id="k-backward"
      data-cmd="backward"
    >
      ▼
      <small>
        S / ↓
      </small>
    </button>

    <span></span>

  </div>


  <div class="hint">

    Use
    <kbd>W</kbd>
    <kbd>A</kbd>
    <kbd>S</kbd>
    <kbd>D</kbd>

    or

    <kbd>↑</kbd>
    <kbd>←</kbd>
    <kbd>↓</kbd>
    <kbd>→</kbd>

    · hold to move, release to stop

  </div>


  <div class="state">

    Status:

    <b id="stateTxt">
      IDLE
    </b>

  </div>

</section>


<!-- ======================================
     CHECKPOINTS
     ====================================== -->

<section class="card">

  <h2>
    Road Checkpoints
  </h2>


  <div id="nodes">

    <div class="empty">
      Waiting for checkpoints…
    </div>

  </div>

</section>


</div>


<div class="foot">

  <b>
    सुरक्षा पहले
  </b>

  · SAFETY FIRST · NMDC BAILADILA

</div>


</div>


<script>


// ==========================================
// DISTANCE SETTINGS
// ==========================================

const DIST_DANGER = 2;

const DIST_CAUTION = 4;

const DIST_MAX = 10;


// ==========================================
// KEYBOARD CONTROLS
// ==========================================

const KEYMAP = {

  arrowup: 'forward',

  w: 'forward',

  arrowdown: 'backward',

  s: 'backward',

  arrowleft: 'left',

  a: 'left',

  arrowright: 'right',

  d: 'right'

};


const LABEL = {

  forward: 'FORWARD',

  backward: 'REVERSE',

  left: 'LEFT',

  right: 'RIGHT'

};


let current = null;

let timer = null;

const held = [];


// ==========================================
// SEND COMMAND
// ==========================================

function send(cmd) {

  fetch('/' + cmd)
    .catch(() => {});

}


// ==========================================
// DRIVE
// ==========================================

function drive(cmd) {

  if (cmd === current)
    return;


  current = cmd;


  clearInterval(timer);

  timer = null;


  document
    .querySelectorAll('.key')
    .forEach(
      k => k.classList.remove('active')
    );


  if (cmd) {

    send(cmd);


    timer =
      setInterval(
        () => send(cmd),
        200
      );


    const el =
      document.getElementById(
        'k-' + cmd
      );


    if (el)
      el.classList.add('active');


    document
      .getElementById('stateTxt')
      .textContent =
      LABEL[cmd];


  } else {

    send('stop');


    document
      .getElementById('stateTxt')
      .textContent =
      'IDLE';

  }

}


// ==========================================
// KEY COMMAND
// ==========================================

function keyCmd() {

  return held.length
    ? KEYMAP[
        held[held.length - 1]
      ]
    : null;

}


// ==========================================
// KEY DOWN
// ==========================================

document.addEventListener(
  'keydown',
  e => {

    const k =
      e.key.toLowerCase();


    if (k === ' ') {

      e.preventDefault();

      held.length = 0;

      drive(null);

      return;

    }


    if (!(k in KEYMAP))
      return;


    e.preventDefault();


    if (!held.includes(k))
      held.push(k);


    drive(keyCmd());

  }
);


// ==========================================
// KEY UP
// ==========================================

document.addEventListener(
  'keyup',
  e => {

    const k =
      e.key.toLowerCase();


    const i =
      held.indexOf(k);


    if (i !== -1)
      held.splice(i, 1);


    if (k in KEYMAP)
      drive(keyCmd());

  }
);


// ==========================================
// WINDOW BLUR
// ==========================================

window.addEventListener(
  'blur',
  () => {

    held.length = 0;

    drive(null);

  }
);


// ==========================================
// BUTTON FOCUS
// ==========================================

document
  .querySelectorAll('button')
  .forEach(
    b =>
      b.addEventListener(
        'focus',
        () => b.blur()
      )
  );


// ==========================================
// TOUCH BUTTONS
// ==========================================

document
  .querySelectorAll('.key')
  .forEach(btn => {

    const cmd =
      btn.dataset.cmd;


    if (cmd === 'stop') {

      btn.addEventListener(
        'click',
        () => {

          held.length = 0;

          drive(null);

        }
      );

      return;

    }


    btn.addEventListener(
      'pointerdown',
      e => {

        e.preventDefault();

        drive(cmd);

      }
    );


    [
      'pointerup',
      'pointerleave',
      'pointercancel'
    ]
    .forEach(
      ev => {

        btn.addEventListener(
          ev,
          () => {

            if (
              current === cmd &&
              !held.length
            ) {

              drive(null);

            }

          }
        );

      }
    );

  });


// ==========================================
// SOUND
// ==========================================

let soundOn = true;

let actx = null;


document
  .getElementById('soundBtn')
  .addEventListener(
    'click',
    () => {

      soundOn = !soundOn;

      document
        .getElementById('soundBtn')
        .textContent =
        soundOn
        ? '🔔 Sound on'
        : '🔕 Sound off';

    }
  );


function beep(times, freq) {

  if (!soundOn)
    return;


  try {

    actx =
      actx ||
      new (
        window.AudioContext ||
        window.webkitAudioContext
      )();


    if (
      actx.state === 'suspended'
    ) {

      actx.resume();

    }


    for (
      let i = 0;
      i < times;
      i++
    ) {

      const o =
        actx.createOscillator();


      const g =
        actx.createGain();


      o.type = 'square';

      o.frequency.value =
        freq;


      g.gain.value =
        0.05;


      o.connect(g);

      g.connect(
        actx.destination
      );


      const t =
        actx.currentTime +
        i * 0.28;


      o.start(t);

      o.stop(
        t + 0.16
      );

    }

  } catch (e) {}

}


// ==========================================
// CLOCK
// ==========================================

function tickClock() {

  document
    .getElementById('clock')
    .textContent =

    new Date()
      .toLocaleTimeString(
        'en-IN',
        {
          hour12: false
        }
      );

}


setInterval(
  tickClock,
  1000
);

tickClock();


// ==========================================
// HELPERS
// ==========================================

function h(
  tag,
  cls,
  text
) {

  const e =
    document.createElement(tag);


  if (cls)
    e.className = cls;


  if (
    text !== undefined
  )
    e.textContent = text;


  return e;

}


function fmt(v, d) {

  return (
    v === null ||
    v === undefined ||
    v < 0
  )
  ? '—'
  : Number(v).toFixed(d);

}


function zoneOf(dist) {

  if (
    dist < DIST_DANGER
  )
    return 2;


  if (
    dist < DIST_CAUTION
  )
    return 1;


  return 0;

}


const ZONE_TXT = [

  'SAFE',

  'CAUTION',

  'DANGER ZONE'

];


// ==========================================
// MESSAGE CLASSIFICATION
// ==========================================

function classify(text) {

  const t =
    text.toLowerCase();


  if (
    /(stop|danger|halt|obstacle|brake|emergency|fog|blast)/
      .test(t)
  ) {

    return {

      cls: 'danger',

      ico: '🛑',

      lvl: 2

    };

  }


  if (
    /(slow|caution|warn|careful|curve|turn|ramp|steep)/
      .test(t)
  ) {

    return {

      cls: 'warn',

      ico: '⚠️',

      lvl: 1

    };

  }


  if (
    /(clear|go|ok|safe|proceed|resume)/
      .test(t)
  ) {

    return {

      cls: 'ok',

      ico: '✅',

      lvl: 0

    };

  }


  return {

    cls: '',

    ico: '📡',

    lvl: 0

  };

}


// ==========================================
// POPUP
// ==========================================

function showToast(
  node,
  text
) {

  const c =
    classify(text);


  const el =
    h(
      'div',
      'toast ' + c.cls
    );


  el.appendChild(
    h(
      'div',
      'ico',
      c.ico
    )
  );


  const body =
    h('div');


  body.appendChild(
    h(
      'div',
      'from',
      'Message from ' + node
    )
  );


  body.appendChild(
    h(
      'div',
      'txt',
      text
    )
  );


  el.appendChild(body);


  document
    .getElementById('toasts')
    .appendChild(el);


  // Sound
  if (c.lvl === 2)

    beep(3, 880);

  else if (c.lvl === 1)

    beep(2, 660);


  // Remove after 4 seconds
  setTimeout(
    () => {

      el.classList.add(
        'hide'
      );


      setTimeout(
        () => el.remove(),
        320
      );

    },
    4000
  );

}


// ==========================================
// LAST MESSAGE IDs
// ==========================================

const lastMsgId = {};


// ==========================================
// STAT ELEMENT
// ==========================================

function stat(
  value,
  label
) {

  const s =
    h(
      'div',
      'stat'
    );


  s.appendChild(
    h(
      'div',
      'v',
      value
    )
  );


  s.appendChild(
    h(
      'div',
      'l',
      label
    )
  );


  return s;

}


// ==========================================
// ROAD ALERT
// ==========================================

function renderAlert(data) {

  let lvl = 0;

  let src =
    'No hazards reported';


  data.nodes.forEach(
    n => {


      // Message priority
      if (
        n.msg &&
        n.msgAge !== null &&
        n.msgAge < 8
      ) {

        const c =
          classify(n.msg);


        if (c.lvl > lvl) {

          lvl = c.lvl;

          src =
            n.name +
            ': ' +
            n.msg;

        }

      }


      // Distance priority
      if (
        n.age !== null &&
        n.age < 3 &&
        n.distance !== null
      ) {

        const z =
          zoneOf(
            n.distance
          );


        if (z > lvl) {

          lvl = z;

          src =
            n.name +
            ' at ' +
            fmt(
              n.distance,
              1
            ) +
            ' m';

        }

      }

    }
  );


  const box =
    document.getElementById(
      'alert'
    );


  box.className =
    'alert ' +
    [
      'ok',
      'warn',
      'danger'
    ][lvl];


  document
    .getElementById('aIco')
    .textContent =
    [
      '✅',
      '⚠️',
      '🛑'
    ][lvl];


  document
    .getElementById('aTxt')
    .textContent =
    [
      'ROAD CLEAR',
      'CAUTION · SLOW DOWN',
      'STOP · HAZARD'
    ][lvl];


  document
    .getElementById('aSrc')
    .textContent =
    src;

}


// ==========================================
// RENDER NODE CARDS
// ==========================================

function render(data) {

  const box =
    document.getElementById(
      'nodes'
    );


  box.innerHTML = '';


  data.nodes.forEach(
    n => {

      const card =
        h(
          'div',
          'node'
        );


      const online =
        (
          n.age !== null &&
          n.age < 3
        );


      // ------------------------------
      // TOP
      // ------------------------------

      const top =
        h(
          'div',
          'node-top'
        );


      const name =
        h(
          'div',
          'node-name'
        );


      name.appendChild(
        h(
          'span',
          'dot' +
          (
            online
            ? ' on'
            : ''
          )
        )
      );


      name.appendChild(
        document.createTextNode(
          n.name
        )
      );


      top.appendChild(name);


      top.appendChild(
        h(
          'div',
          'age',
          n.age === null
          ? 'no data yet'
          : (
            'seen ' +
            fmt(n.age, 1) +
            's ago'
          )
        )
      );


      card.appendChild(top);


      // ------------------------------
      // NUMBERS
      // ------------------------------

      const stats =
        h(
          'div',
          'stats'
        );


      stats.appendChild(
        stat(
          n.rssi === null
          ? '—'
          : n.rssi + ' dBm',
          'RSSI'
        )
      );


      stats.appendChild(
        stat(
          n.distance === null
          ? '—'
          : fmt(
              n.distance,
              2
            ) + ' m',
          'Distance'
        )
      );


      stats.appendChild(
        stat(
          n.eta === null
          ? '—'
          : fmt(
              n.eta,
              1
            ) + ' s',
          'ETA'
        )
      );


      card.appendChild(
        stats
      );


      // ------------------------------
      // PROXIMITY
      // ------------------------------

      const prox =
        h(
          'div',
          'prox'
        );


      const bar =
        h(
          'div',
          'bar'
        );


      const fill =
        h(
          'div',
          'fill'
        );


      const zrow =
        h(
          'div',
          'zrow'
        );


      zrow.appendChild(
        h(
          'span',
          '',
          'PROXIMITY'
        )
      );


      if (
        n.distance !== null
      ) {

        const z =
          zoneOf(
            n.distance
          );


        const pct =
          Math.max(
            0,
            Math.min(
              1,
              1 -
              n.distance /
              DIST_MAX
            )
          ) * 100;


        fill.className =
          'fill z' + z;


        fill.style.width =
          pct + '%';


        zrow.appendChild(
          h(
            'span',
            'z' + z,
            ZONE_TXT[z]
          )
        );


      } else {

        zrow.appendChild(
          h(
            'span',
            '',
            'NO SIGNAL'
          )
        );

      }


      bar.appendChild(
        fill
      );


      prox.appendChild(
        bar
      );


      prox.appendChild(
        zrow
      );


      card.appendChild(
        prox
      );


      // ------------------------------
      // LAST MESSAGE
      // ------------------------------

      const lm =
        h(
          'div',
          'lastmsg'
        );


      if (n.msg) {

        lm.appendChild(
          document.createTextNode(
            'Last message: '
          )
        );


        lm.appendChild(
          h(
            'b',
            'l' +
            classify(
              n.msg
            ).lvl,
            n.msg
          )
        );


      } else {

        lm.textContent =
          'No message received';

      }


      card.appendChild(lm);


      box.appendChild(card);


      // ------------------------------
      // NEW MESSAGE POPUP
      // ------------------------------

      if (
        n.msg &&
        n.msgId !==
        lastMsgId[n.name]
      ) {

        if (
          n.msgAge !== null &&
          n.msgAge < 5
        ) {

          showToast(
            n.name,
            n.msg
          );

        }


        lastMsgId[n.name] =
          n.msgId;

      }

    }
  );


  renderAlert(data);


  document
    .getElementById('reached')
    .classList.toggle(
      'show',
      data.reached === true
    );

}


// ==========================================
// STATUS POLLING
// ==========================================

function poll() {

  fetch('/status')

    .then(
      r => r.json()
    )

    .then(
      data => {

        document
          .getElementById(
            'linkDot'
          )
          .className =
          'dot on';


        document
          .getElementById(
            'linkTxt'
          )
          .textContent =
          'Connected';


        render(data);

      }
    )

    .catch(
      () => {

        document
          .getElementById(
            'linkDot'
          )
          .className =
          'dot off';


        document
          .getElementById(
            'linkTxt'
          )
          .textContent =
          'Disconnected';

      }
    );

}


setInterval(
  poll,
  400
);


poll();

</script>

</body>

</html>

)rawliteral";


// ==========================================
// JSON ESCAPE
// ==========================================

String jsonEscape(
  const char* s
) {

  String out;


  for (; *s; s++) {

    if (
      *s == '"' ||
      *s == '\\'
    ) {

      out += '\\';

      out += *s;

    }

    else if (
      (uint8_t)*s < 0x20
    ) {

      out += ' ';

    }

    else {

      out += *s;

    }

  }


  return out;

}


// ==========================================
// STATUS ENDPOINT
// ==========================================

void handleStatus() {

  unsigned long now =
    millis();


  bool reached =
    hasMoved &&
    (
      now - lastMoveCmd >
      DESTINATION_TIMEOUT_MS
    );


  String json =
    "{\"reached\":";


  json +=
    reached
    ? "true"
    : "false";


  json +=
    ",\"nodes\":[";


  for (
    int i = 0;
    i < NUM_KNOWN_PEERS;
    i++
  ) {

    NodeState &n =
      nodeStates[i];


    if (i > 0)
      json += ",";


    json += "{";


    // ------------------------------
    // NODE NAME
    // ------------------------------

    json +=
      "\"name\":\"" +
      jsonEscape(
        knownPeers[i].name
      ) +
      "\",";


    // ------------------------------
    // RSSI / DISTANCE / ETA
    // ------------------------------

    if (n.everSeen) {

      float ageSec =
        (
          now -
          n.lastSeen
        ) / 1000.0f;


      json +=
        "\"rssi\":" +
        String(n.rssi) +
        ",";


      json +=
        "\"distance\":" +
        String(
          n.distance,
          2
        ) +
        ",";


      json +=
        "\"eta\":" +
        String(
          n.eta,
          1
        ) +
        ",";


      json +=
        "\"age\":" +
        String(
          ageSec,
          1
        ) +
        ",";

    }

    else {

      json +=
        "\"rssi\":null,"
        "\"distance\":null,"
        "\"eta\":null,"
        "\"age\":null,";

    }


    // ------------------------------
    // MESSAGE
    // ------------------------------

    if (
      n.msg[0] != '\0'
    ) {

      float msgAge =
        (
          now -
          n.msgTime
        ) / 1000.0f;


      json +=
        "\"msg\":\"" +
        jsonEscape(
          n.msg
        ) +
        "\",";


      json +=
        "\"msgId\":" +
        String(
          n.msgId
        ) +
        ",";


      json +=
        "\"msgAge\":" +
        String(
          msgAge,
          1
        );

    }

    else {

      json +=
        "\"msg\":null,"
        "\"msgId\":0,"
        "\"msgAge\":null";

    }


    json += "}";

  }


  json += "]}";


  server.send(
    200,
    "application/json",
    json
  );

}


// ==========================================
// WEB HANDLERS
// ==========================================

void handleRoot() {

  server.send(
    200,
    "text/html",
    INDEX_HTML
  );

}


void handleForward() {

  markMoveCommand();

  forward();

  server.send(
    200,
    "text/plain",
    "FORWARD"
  );

}


void handleBackward() {

  markMoveCommand();

  backward();

  server.send(
    200,
    "text/plain",
    "BACKWARD"
  );

}


void handleLeft() {

  markMoveCommand();

  turnLeft();

  server.send(
    200,
    "text/plain",
    "LEFT"
  );

}


void handleRight() {

  markMoveCommand();

  turnRight();

  server.send(
    200,
    "text/plain",
    "RIGHT"
  );

}


void handleStop() {

  stopMotors();

  motorsRunning = false;

  server.send(
    200,
    "text/plain",
    "STOP"
  );

}


// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);


  // ========================================
  // MOTOR PINS
  // ========================================

  pinMode(
    ENA,
    OUTPUT
  );

  pinMode(
    IN1,
    OUTPUT
  );

  pinMode(
    IN2,
    OUTPUT
  );


  pinMode(
    ENB,
    OUTPUT
  );

  pinMode(
    IN3,
    OUTPUT
  );

  pinMode(
    IN4,
    OUTPUT
  );


  stopMotors();


  // ========================================
  // WIFI ACCESS POINT
  // ========================================

  WiFi.mode(
    WIFI_AP_STA
  );


  delay(500);


  WiFi.softAP(
    ssid,
    password,
    WIFI_CHANNEL
  );


  Serial.println();

  Serial.println(
    "============================"
  );

  Serial.println(
    "ESP32 ROBOT"
  );

  Serial.println(
    "============================"
  );


  Serial.print(
    "WiFi Name: "
  );

  Serial.println(ssid);


  Serial.print(
    "IP Address: "
  );

  Serial.println(
    WiFi.softAPIP()
  );


  // ========================================
  // ESP-NOW
  // ========================================

  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );


  if (
    esp_now_init() != ESP_OK
  ) {

    Serial.println(
      "ESP-NOW init failed"
    );

  }

  else {

    esp_now_register_send_cb(
      OnDataSent
    );

    esp_now_register_recv_cb(
      OnDataRecv
    );


    for (
      int i = 0;
      i < NUM_KNOWN_PEERS;
      i++
    ) {

      esp_now_peer_info_t peerInfo = {};


      memcpy(
        peerInfo.peer_addr,
        knownPeers[i].mac,
        6
      );


      peerInfo.channel =
        WIFI_CHANNEL;


      peerInfo.encrypt =
        false;


      if (
        esp_now_add_peer(
          &peerInfo
        ) != ESP_OK
      ) {

        Serial.print(
          "Failed to add peer: "
        );

        Serial.println(
          knownPeers[i].name
        );

      }


      memset(
        &nodeStates[i],
        0,
        sizeof(NodeState)
      );


      nodeStates[i].everSeen =
        false;

    }


    memset(
      &outgoingPing,
      0,
      sizeof(outgoingPing)
    );


    strncpy(
      outgoingPing.nodeName,
      "CAR",
      sizeof(
        outgoingPing.nodeName
      ) - 1
    );


    Serial.println(
      "ESP-NOW ready, watching for track nodes"
    );

  }


  // ========================================
  // WEB SERVER
  // ========================================

  server.on(
    "/",
    handleRoot
  );


  server.on(
    "/forward",
    handleForward
  );


  server.on(
    "/backward",
    handleBackward
  );


  server.on(
    "/left",
    handleLeft
  );


  server.on(
    "/right",
    handleRight
  );


  server.on(
    "/stop",
    handleStop
  );


  server.on(
    "/status",
    handleStatus
  );


  server.begin();


  Serial.println(
    "Web server started"
  );

}


// ==========================================
// LOOP
// ==========================================

void loop() {

  server.handleClient();


  // ========================================
  // FAILSAFE
  // ========================================

  if (
    motorsRunning &&
    (
      millis() -
      lastMoveCmd >
      FAILSAFE_TIMEOUT_MS
    )
  ) {

    stopMotors();

    motorsRunning =
      false;

  }


  // ========================================
  // PING TRACK NODES
  // ========================================

  static unsigned long lastPing = 0;


  if (
    millis() -
    lastPing >= 1000
  ) {

    lastPing =
      millis();


    for (
      int i = 0;
      i < NUM_KNOWN_PEERS;
      i++
    ) {

      esp_now_send(
        knownPeers[i].mac,
        (uint8_t*)&outgoingPing,
        sizeof(outgoingPing)
      );

    }

  }

}
