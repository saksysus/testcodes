// ============================================================
// TRINETRA — C1 VEHICLE NODE
// ============================================================
// C1 is the moving ESP32.
//
// Fixed nodes:
//   NODE1
//   NODE2
//
// C1 receives both beacons and uses RSSI to estimate
// relative position between the two fixed nodes.
//
// IMPORTANT:
// RSSI is noisy. This code filters RSSI using a moving average.
// The position is an APPROXIMATION, not precision ranging.
// ============================================================

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// ============================================================
// CONFIGURATION
// ============================================================

// Physical distance between NODE1 and NODE2.
// CHANGE THIS to your actual test distance.

const float NODE_DISTANCE = 0.5;   // metres

// Vehicle speed.
// Change this according to your vehicle.

const float VEHICLE_SPEED_KMH = 2.0;

// ESP-NOW channel.
// Must match NODE1 and NODE2.

#define WIFI_CHANNEL 1

// ============================================================
// PACKET FORMAT
// ============================================================

typedef struct {
  char nodeName[16];
} BeaconPacket;

// ============================================================
// RSSI STORAGE
// ============================================================

volatile int node1RSSI = -100;
volatile int node2RSSI = -100;

volatile bool node1Seen = false;
volatile bool node2Seen = false;

volatile unsigned long node1LastSeen = 0;
volatile unsigned long node2LastSeen = 0;

// ============================================================
// RSSI FILTER
// ============================================================

#define FILTER_SIZE 8

int node1Samples[FILTER_SIZE];
int node2Samples[FILTER_SIZE];

int node1Index = 0;
int node2Index = 0;

int node1Count = 0;
int node2Count = 0;

// ============================================================
// CALCULATE MOVING AVERAGE
// ============================================================

float calculateAverage(int samples[], int count) {

  if (count == 0) {
    return -100;
  }

  long total = 0;

  for (int i = 0; i < count; i++) {
    total += samples[i];
  }

  return (float)total / count;
}

// ============================================================
// ADD NODE1 RSSI SAMPLE
// ============================================================

void addNode1Sample(int rssi) {

  node1Samples[node1Index] = rssi;

  node1Index++;

  if (node1Index >= FILTER_SIZE) {
    node1Index = 0;
  }

  if (node1Count < FILTER_SIZE) {
    node1Count++;
  }
}

// ============================================================
// ADD NODE2 RSSI SAMPLE
// ============================================================

void addNode2Sample(int rssi) {

  node2Samples[node2Index] = rssi;

  node2Index++;

  if (node2Index >= FILTER_SIZE) {
    node2Index = 0;
  }

  if (node2Count < FILTER_SIZE) {
    node2Count++;
  }
}

// ============================================================
// ESP-NOW RECEIVE CALLBACK
// ============================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {

  if (len < sizeof(BeaconPacket)) {
    return;
  }

  BeaconPacket packet;

  memcpy(
    &packet,
    incomingData,
    sizeof(packet)
  );

  int rssi = info->rx_ctrl->rssi;

  // ----------------------------------------------------------
  // NODE1
  // ----------------------------------------------------------

  if (strcmp(packet.nodeName, "NODE1") == 0) {

    node1RSSI = rssi;
    node1Seen = true;
    node1LastSeen = millis();

    addNode1Sample(rssi);
  }

  // ----------------------------------------------------------
  // NODE2
  // ----------------------------------------------------------

  else if (strcmp(packet.nodeName, "NODE2") == 0) {

    node2RSSI = rssi;
    node2Seen = true;
    node2LastSeen = millis();

    addNode2Sample(rssi);
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Force same ESP-NOW channel
  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  // ----------------------------------------------------------
  // ESP-NOW
  // ----------------------------------------------------------

  if (esp_now_init() != ESP_OK) {

    Serial.println();
    Serial.println("ESP-NOW INITIALIZATION FAILED");

    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // ----------------------------------------------------------
  // STARTUP MESSAGE
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("======================================");
  Serial.println("          TRINETRA - C1");
  Serial.println("        VEHICLE ESP32 NODE");
  Serial.println("======================================");

  Serial.print("C1 MAC             : ");
  Serial.println(WiFi.macAddress());

  Serial.print("NODE1 <-> NODE2    : ");
  Serial.print(NODE_DISTANCE);
  Serial.println(" m");

  Serial.print("Vehicle speed      : ");
  Serial.print(VEHICLE_SPEED_KMH);
  Serial.println(" km/h");

  Serial.print("ESP-NOW channel    : ");
  Serial.println(WIFI_CHANNEL);

  Serial.println("--------------------------------------");
  Serial.println("Waiting for NODE1 and NODE2...");
  Serial.println("======================================");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  unsigned long currentTime = millis();

  // ----------------------------------------------------------
  // CHECK NODE TIMEOUT
  // ----------------------------------------------------------

  bool node1Active =
    node1Seen &&
    (currentTime - node1LastSeen < 3000);

  bool node2Active =
    node2Seen &&
    (currentTime - node2LastSeen < 3000);

  // ----------------------------------------------------------
  // GET FILTERED RSSI
  // ----------------------------------------------------------

  float filteredNode1RSSI =
    calculateAverage(node1Samples, node1Count);

  float filteredNode2RSSI =
    calculateAverage(node2Samples, node2Count);

  // ==========================================================
  // NOTHING DETECTED
  // ==========================================================

  if (!node1Active && !node2Active) {

    Serial.println();
    Serial.println("======================================");
    Serial.println("C1 STATUS: SEARCHING FOR NODES...");
    Serial.println("======================================");

    delay(1000);

    return;
  }

  // ==========================================================
  // DISPLAY NODE1
  // ==========================================================

  Serial.println();
  Serial.println("======================================");
  Serial.println("             C1 STATUS");
  Serial.println("======================================");

  if (node1Active) {

    Serial.print("NODE1 RSSI         : ");
    Serial.print(filteredNode1RSSI, 1);
    Serial.println(" dBm");

  } else {

    Serial.println("NODE1              : NOT DETECTED");
  }

  // ==========================================================
  // DISPLAY NODE2
  // ==========================================================

  if (node2Active) {

    Serial.print("NODE2 RSSI         : ");
    Serial.print(filteredNode2RSSI, 1);
    Serial.println(" dBm");

  } else {

    Serial.println("NODE2              : NOT DETECTED");
  }

  // ==========================================================
  // BOTH NODES AVAILABLE
  // ==========================================================

  if (node1Active && node2Active) {

    // --------------------------------------------------------
    // DETERMINE WHICH NODE IS CLOSER
    // --------------------------------------------------------

    if (filteredNode1RSSI > filteredNode2RSSI) {

      Serial.println();
      Serial.println("NEXT NODE          : NODE1");

    } else {

      Serial.println();
      Serial.println("NEXT NODE          : NODE2");
    }

    // --------------------------------------------------------
    // RSSI DIFFERENCE
    // --------------------------------------------------------

    float rssiDifference =
      filteredNode1RSSI - filteredNode2RSSI;

    Serial.print("RSSI difference    : ");
    Serial.print(rssiDifference, 1);
    Serial.println(" dB");

    // --------------------------------------------------------
    // APPROXIMATE POSITION
    // --------------------------------------------------------
    //
    // If NODE1 signal is stronger:
    //     C1 is closer to NODE1.
    //
    // If NODE2 signal is stronger:
    //     C1 is closer to NODE2.
    //
    // We normalize the RSSI difference into a position
    // between the two nodes.
    //
    // This is an APPROXIMATION and should not be treated
    // as precision ranging.
    // --------------------------------------------------------

    float ratio =
      (filteredNode2RSSI) /
      (filteredNode2RSSI + filteredNode1RSSI);

    // Instead of relying on the raw ratio above,
    // use RSSI difference to create a bounded position.

    float positionRatio =
      0.5 - (rssiDifference / 40.0);

    // Keep position between 0 and 1

    if (positionRatio < 0.0) {
      positionRatio = 0.0;
    }

    if (positionRatio > 1.0) {
      positionRatio = 1.0;
    }

    float positionFromNode1 =
      positionRatio * NODE_DISTANCE;

    float positionFromNode2 =
      NODE_DISTANCE - positionFromNode1;

    // --------------------------------------------------------
    // DISPLAY POSITION
    // --------------------------------------------------------

    Serial.println();
    Serial.println("------------- POSITION ---------------");

    Serial.print("From NODE1         : ");
    Serial.print(positionFromNode1, 1);
    Serial.println(" m");

    Serial.print("From NODE2         : ");
    Serial.print(positionFromNode2, 1);
    Serial.println(" m");

    // --------------------------------------------------------
    // ETA
    // --------------------------------------------------------

    float speedMS =
      VEHICLE_SPEED_KMH / 3.6;

    if (speedMS > 0) {

      float distanceToNextNode;

      if (positionFromNode1 < positionFromNode2) {

        distanceToNextNode =
          positionFromNode1;

      } else {

        distanceToNextNode =
          positionFromNode2;
      }

      float eta =
        distanceToNextNode / speedMS;

      Serial.println();
      Serial.println("--------------- ETA -----------------");

      Serial.print("Distance to next  : ");
      Serial.print(distanceToNextNode, 1);
      Serial.println(" m");

      Serial.print("Vehicle speed     : ");
      Serial.print(VEHICLE_SPEED_KMH, 1);
      Serial.println(" km/h");

      Serial.print("ETA               : ");
      Serial.print(eta, 1);
      Serial.println(" seconds");
    }

  }

  // ==========================================================
  // ONLY NODE1 AVAILABLE
  // ==========================================================

  else if (node1Active) {

    Serial.println();
    Serial.println("NEXT NODE          : NODE1");
    Serial.println("NODE2              : OUT OF RANGE");
  }

  // ==========================================================
  // ONLY NODE2 AVAILABLE
  // ==========================================================

  else if (node2Active) {

    Serial.println();
    Serial.println("NEXT NODE          : NODE2");
    Serial.println("NODE1              : OUT OF RANGE");
  }

  Serial.println("======================================");

  delay(1000);
}