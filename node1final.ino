// ============================================================
// TRINETRA — FIXED TRACK NODE 1
// Ultrasonic + ESP-NOW sender for CAR 1
// ============================================================
// BEHAVIOUR
// 1. HC-SR04 detects a car at <= 0.30 m.
// 2. Send: "Curve 17 Ahead. SLOW DOWN"
// 3. Do not send Slope until Curve delivery succeeds.
// 4. When the car leaves the ultrasonic detection zone,
//    wait a few seconds.
// 5. Send: "Slope Ahead"
// 6. After Slope delivery, reset the cycle so the next car
//    can trigger a fresh Curve -> Slope sequence.
//
// CAR 1 compatibility:
// - Same packet: { uint8_t messageType; char message[64]; }
// - MSG_CURVE = 1
// - MSG_SLOPE = 2
// - Wi-Fi / ESP-NOW channel = 1
// - CAR 1 MAC = 00:70:07:E1:BC:EC
//
// IMPORTANT: Upload this code only to NODE1.
// Do NOT change your CAR 1 sketch.
// ============================================================

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// ============================================================
// CONFIGURATION
// ============================================================

const char* NODE_NAME = "NODE1";

#define WIFI_CHANNEL 1

// HC-SR04 pins
#define TRIG_PIN 18
#define ECHO_PIN 19

// Car detection threshold
#define DETECTION_DISTANCE 0.30f   // metres

// Number of consecutive readings outside the detection zone
// required before we consider the car to have left.
#define CLEAR_REQUIRED 3

// Delay AFTER the car has left the sensor before Slope is sent.
// 5000 ms also gives the CAR 1 Curve popup time to disappear
// before the next popup normally arrives.
#define SLOPE_DELAY_AFTER_EXIT_MS 5000UL

// Retry interval if ESP-NOW delivery fails.
#define SEND_RETRY_MS 500UL

// Time to keep the cycle visibly complete before re-arming.
#define RESET_AFTER_SLOPE_MS 1000UL

// Ultrasonic echo timeout: 30 ms is more than enough for this range.
#define ECHO_TIMEOUT_US 30000UL

// ============================================================
// CAR 1 MAC ADDRESS
// ============================================================

uint8_t carMAC[] = {
  0x00,
  0x70,
  0x07,
  0xE1,
  0xBC,
  0xEC
};

// ============================================================
// MESSAGE TYPES — MUST MATCH CAR 1
// ============================================================

#define MSG_CURVE  1
#define MSG_SLOPE  2

// ============================================================
// SHARED ALERT PACKET — MUST MATCH CAR 1
// sizeof(AlertPacket) = 65 bytes on ESP32
// ============================================================

typedef struct {
  uint8_t messageType;
  char message[64];
} AlertPacket;

// ============================================================
// ALERT / DETECTION STATE
// ============================================================

bool carDetected = false;
bool carLeftSensor = false;
bool curveDelivered = false;
bool slopeDelivered = false;

int clearCount = 0;

unsigned long carLeftAt = 0;
unsigned long slopeDeliveredAt = 0;
unsigned long nextRetryAt = 0;

// ============================================================
// TRANSMISSION STATE
// Only one ESP-NOW packet is allowed to be in flight at a time.
// The callback only records the result; actual next transmission
// is started from loop(), which is safer than sending from the
// Wi-Fi callback.
// ============================================================

enum TxType : uint8_t {
  TX_NONE = 0,
  TX_CURVE,
  TX_SLOPE
};

volatile bool txBusy = false;
volatile TxType txType = TX_NONE;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void sendCurveAlert();
void sendSlopeAlert();
void serviceAlerts();
void resetAlertCycle();
void checkUltrasonic();
float getDistance();
void printDashboard();

// ============================================================
// ESP-NOW SEND CALLBACK
// Compatible with Arduino-ESP32 3.x
// ============================================================

void OnDataSent(
  const wifi_tx_info_t *tx_info,
  esp_now_send_status_t status
) {
  (void)tx_info;

  TxType completedType = txType;

  // Mark the transmission as no longer in flight.
  txBusy = false;
  txType = TX_NONE;

  if (status == ESP_NOW_SEND_SUCCESS) {

    if (completedType == TX_CURVE) {
      curveDelivered = true;
      Serial.println("[ESP-NOW] Curve delivery SUCCESS");
      Serial.println("[NODE1] Curve alert delivered to CAR 1");
    }

    else if (completedType == TX_SLOPE) {
      slopeDelivered = true;
      slopeDeliveredAt = millis();
      Serial.println("[ESP-NOW] Slope delivery SUCCESS");
      Serial.println("[NODE1] Slope alert delivered to CAR 1");
    }
  }

  else {
    // Do not lose the alert after a failed transmission.
    // serviceAlerts() will retry automatically.
    nextRetryAt = millis() + SEND_RETRY_MS;

    if (completedType == TX_CURVE) {
      Serial.println("[ESP-NOW] Curve delivery FAILED -> retrying");
    }

    else if (completedType == TX_SLOPE) {
      Serial.println("[ESP-NOW] Slope delivery FAILED -> retrying");
    }
  }
}

// ============================================================
// SEND CURVE ALERT
// ============================================================

void sendCurveAlert() {

  if (!carDetected) return;
  if (curveDelivered) return;
  if (txBusy) return;

  AlertPacket packet = {};
  packet.messageType = MSG_CURVE;

  strncpy(
    packet.message,
    "Curve 17 Ahead. SLOW DOWN",
    sizeof(packet.message) - 1
  );
  packet.message[sizeof(packet.message) - 1] = '\0';

  txType = TX_CURVE;
  txBusy = true;

  esp_err_t result = esp_now_send(
    carMAC,
    reinterpret_cast<uint8_t*>(&packet),
    sizeof(packet)
  );

  if (result != ESP_OK) {
    txBusy = false;
    txType = TX_NONE;
    nextRetryAt = millis() + SEND_RETRY_MS;

    Serial.print("[NODE1] Curve esp_now_send() error: ");
    Serial.println((int)result);
    return;
  }

  Serial.println();
  Serial.println(">>> CURVE ALERT TRANSMITTING <<<");
  Serial.println("Curve 17 Ahead. SLOW DOWN");
}

// ============================================================
// SEND SLOPE ALERT
// ============================================================

void sendSlopeAlert() {

  // Never send Slope before Curve has been delivered.
  if (!carDetected) return;
  if (!carLeftSensor) return;
  if (!curveDelivered) return;
  if (slopeDelivered) return;
  if (txBusy) return;

  AlertPacket packet = {};
  packet.messageType = MSG_SLOPE;

  strncpy(
    packet.message,
    "Slope Ahead",
    sizeof(packet.message) - 1
  );
  packet.message[sizeof(packet.message) - 1] = '\0';

  txType = TX_SLOPE;
  txBusy = true;

  esp_err_t result = esp_now_send(
    carMAC,
    reinterpret_cast<uint8_t*>(&packet),
    sizeof(packet)
  );

  if (result != ESP_OK) {
    txBusy = false;
    txType = TX_NONE;
    nextRetryAt = millis() + SEND_RETRY_MS;

    Serial.print("[NODE1] Slope esp_now_send() error: ");
    Serial.println((int)result);
    return;
  }

  Serial.println();
  Serial.println(">>> SLOPE ALERT TRANSMITTING <<<");
  Serial.println("Slope Ahead");
}

// ============================================================
// ULTRASONIC DISTANCE
// ============================================================

float getDistance() {

  // Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    ECHO_TIMEOUT_US
  );

  if (duration == 0) {
    return -1.0f;
  }

  float distance_cm = duration * 0.0343f / 2.0f;
  return distance_cm / 100.0f;
}

// ============================================================
// CHECK ULTRASONIC
// ============================================================

void checkUltrasonic() {

  float distance = getDistance();

  bool carInRange = (
    distance > 0.0f &&
    distance <= DETECTION_DISTANCE
  );

  // ==========================================================
  // CAR DETECTED / STILL INSIDE SENSOR ZONE
  // ==========================================================

  if (carInRange) {

    clearCount = 0;

    // If the vehicle comes back before the old cycle was reset,
    // keep it as the same vehicle and do not issue a second curve.
    if (!carDetected) {
      carDetected = true;
      carLeftSensor = false;
      curveDelivered = false;
      slopeDelivered = false;
      slopeDeliveredAt = 0;
      nextRetryAt = 0;

      Serial.println();
      Serial.println("############################################");
      Serial.println("[NODE1] CAR 1 DETECTED");
      Serial.print("[NODE1] Distance: ");
      Serial.print(distance, 2);
      Serial.println(" m");
      Serial.println("[NODE1] Curve alert queued");
      Serial.println("############################################");
    }

    return;
  }

  // ==========================================================
  // CAR OUTSIDE SENSOR ZONE
  // ==========================================================

  if (carDetected && !carLeftSensor) {

    clearCount++;

    if (clearCount >= CLEAR_REQUIRED) {

      carLeftSensor = true;
      carLeftAt = millis();

      Serial.println();
      Serial.println("[NODE1] CAR 1 LEFT ULTRASONIC LOS");
      Serial.print("[NODE1] Waiting ");
      Serial.print(SLOPE_DELAY_AFTER_EXIT_MS / 1000UL);
      Serial.println(" s before Slope alert...");

      if (!curveDelivered) {
        Serial.println("[NODE1] Curve not delivered yet -> waiting for Curve delivery before Slope");
      }
    }
  }
}

// ============================================================
// ALERT STATE MACHINE
// ============================================================

void serviceAlerts() {

  unsigned long now = millis();

  // A packet is currently being transmitted.
  if (txBusy) return;

  // ----------------------------------------------------------
  // 1) Curve must be delivered first.
  // ----------------------------------------------------------

  if (carDetected && !curveDelivered) {

    if ((long)(now - nextRetryAt) >= 0) {
      sendCurveAlert();
    }

    return;
  }

  // ----------------------------------------------------------
  // 2) After the car leaves, wait before sending Slope.
  // ----------------------------------------------------------

  if (
    carDetected &&
    carLeftSensor &&
    curveDelivered &&
    !slopeDelivered &&
    (now - carLeftAt >= SLOPE_DELAY_AFTER_EXIT_MS) &&
    ((long)(now - nextRetryAt) >= 0)
  ) {
    sendSlopeAlert();
    return;
  }

  // ----------------------------------------------------------
  // 3) Once Slope is delivered, re-arm for the next car.
  // ----------------------------------------------------------

  if (
    carDetected &&
    carLeftSensor &&
    slopeDelivered &&
    (now - slopeDeliveredAt >= RESET_AFTER_SLOPE_MS)
  ) {
    Serial.println();
    Serial.println("[NODE1] Alert sequence complete");
    Serial.println("[NODE1] NODE1 is re-armed for the next car");
    resetAlertCycle();
  }
}

// ============================================================
// RESET ALERT CYCLE
// ============================================================

void resetAlertCycle() {

  carDetected = false;
  carLeftSensor = false;
  curveDelivered = false;
  slopeDelivered = false;

  clearCount = 0;
  carLeftAt = 0;
  slopeDeliveredAt = 0;
  nextRetryAt = 0;
}

// ============================================================
// SERIAL DASHBOARD
// ============================================================

void printDashboard() {

  static unsigned long lastDashboard = 0;

  if (millis() - lastDashboard < 2000UL) {
    return;
  }

  lastDashboard = millis();

  float distance = getDistance();

  Serial.println();
  Serial.println("======================================================");
  Serial.println("              TRINETRA | NODE 1");
  Serial.println("======================================================");

  Serial.println();
  Serial.println("NODE STATUS");
  Serial.println("------------------------------------------------------");

  Serial.print("Node          : ");
  Serial.println(NODE_NAME);

  Serial.print("Ultrasonic    : ");
  if (distance > 0.0f) {
    Serial.print(distance, 2);
    Serial.println(" m");
  } else {
    Serial.println("CLEAR");
  }

  Serial.print("CAR 1         : ");
  Serial.println(carDetected ? "DETECTED" : "NOT DETECTED");

  Serial.println();
  Serial.println("ALERT STATUS");
  Serial.println("------------------------------------------------------");

  Serial.print("Curve 17      : ");
  if (curveDelivered) {
    Serial.println("DELIVERED");
  } else if (txBusy && txType == TX_CURVE) {
    Serial.println("TRANSMITTING");
  } else {
    Serial.println("WAITING / RETRYING");
  }

  Serial.print("Slope         : ");
  if (slopeDelivered) {
    Serial.println("DELIVERED");
  } else if (txBusy && txType == TX_SLOPE) {
    Serial.println("TRANSMITTING");
  } else if (carLeftSensor && curveDelivered) {
    unsigned long elapsed = millis() - carLeftAt;
    if (elapsed < SLOPE_DELAY_AFTER_EXIT_MS) {
      Serial.print("WAITING ");
      Serial.print((SLOPE_DELAY_AFTER_EXIT_MS - elapsed) / 1000UL);
      Serial.println(" s");
    } else {
      Serial.println("READY");
    }
  } else {
    Serial.println("WAITING");
  }

  Serial.println();
  Serial.print("Clear count   : ");
  Serial.println(clearCount);

  Serial.print("TX busy       : ");
  Serial.println(txBusy ? "YES" : "NO");

  Serial.println("======================================================");
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  // ----------------------------------------------------------
  // Ultrasonic
  // ----------------------------------------------------------

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // ----------------------------------------------------------
  // Wi-Fi / ESP-NOW
  // ----------------------------------------------------------

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();

  esp_err_t channelResult = esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  if (channelResult != ESP_OK) {
    Serial.print("[NODE1] Failed to set Wi-Fi channel: ");
    Serial.println((int)channelResult);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[NODE1] ESP-NOW initialization FAILED");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // ----------------------------------------------------------
  // Add CAR 1 peer
  // ----------------------------------------------------------

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, carMAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  esp_err_t peerResult = esp_now_add_peer(&peerInfo);

  if (
    peerResult != ESP_OK &&
    peerResult != ESP_ERR_ESPNOW_EXIST
  ) {
    Serial.print("[NODE1] Failed to add CAR 1 peer: ");
    Serial.println((int)peerResult);
  }

  // ----------------------------------------------------------
  // Startup information
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("======================================================");
  Serial.println("              TRINETRA | NODE 1");
  Serial.println("======================================================");

  Serial.print("Node MAC      : ");
  Serial.println(WiFi.macAddress());

  Serial.println("Destination   : CAR 1");
  Serial.println("CAR MAC       : 00:70:07:E1:BC:EC");

  Serial.print("ESP-NOW       : Channel ");
  Serial.println(WIFI_CHANNEL);

  Serial.print("Detection     : <= ");
  Serial.print(DETECTION_DISTANCE, 2);
  Serial.println(" m");

  Serial.print("Clear filter  : ");
  Serial.print(CLEAR_REQUIRED);
  Serial.println(" readings");

  Serial.print("Slope delay   : ");
  Serial.print(SLOPE_DELAY_AFTER_EXIT_MS / 1000UL);
  Serial.println(" s after car leaves");

  Serial.println();
  Serial.println("ALERT 1       : Curve 17 Ahead. SLOW DOWN");
  Serial.println("ALERT 2       : Slope Ahead");

  Serial.println();
  Serial.println("STATUS        : ACTIVE");
  Serial.println("======================================================");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // Read ultrasonic and update vehicle state.
  checkUltrasonic();

  // Handle Curve -> wait for delivery -> wait after exit -> Slope.
  serviceAlerts();

  // Serial-only status display.
  printDashboard();

  // Small delay so the ultrasonic sensor is not hammered continuously.
  delay(100);
}
