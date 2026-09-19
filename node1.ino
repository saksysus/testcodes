// ============================================================
// TRINETRA — FIXED TRACK NODE
// ============================================================
// Flash this code to BOTH fixed ESP32 nodes.
//
// NODE 1:
//   NODE_NAME = "NODE1"
//
// NODE 2:
//   NODE_NAME = "NODE2"
//
// These nodes continuously broadcast their identity.
// C1 receives the packets and reads RSSI.
// ============================================================

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// ============================================================
// CONFIGURATION
// ============================================================

const char* NODE_NAME = "NODE1";   // CHANGE TO "NODE2" ON SECOND ESP32

#define WIFI_CHANNEL 1

// ============================================================
// PACKET FORMAT
// ============================================================

typedef struct {
  char nodeName[16];
} BeaconPacket;

BeaconPacket outgoing;

uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// ============================================================
// SEND CALLBACK
// ============================================================

void OnDataSent(
  const wifi_tx_info_t *tx_info,
  esp_now_send_status_t status
) {
  Serial.print("[");
  Serial.print(NODE_NAME);
  Serial.print("] Beacon: ");

  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("SENT");
  } else {
    Serial.println("FAILED");
  }
}

// ============================================================
// RECEIVE CALLBACK
// ============================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {
  if (len < sizeof(BeaconPacket)) {
    return;
  }

  BeaconPacket incoming;
  memcpy(&incoming, incomingData, sizeof(incoming));

  Serial.print("[");
  Serial.print(NODE_NAME);
  Serial.print("] Received from ");
  Serial.print(incoming.nodeName);

  Serial.print(" | RSSI: ");
  Serial.print(info->rx_ctrl->rssi);
  Serial.println(" dBm");
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  // Wi-Fi station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Force ESP-NOW onto same channel
  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  // Start ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization FAILED");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    broadcastAddress,
    6
  );

  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    return;
  }

  // Copy node name into packet
  memset(&outgoing, 0, sizeof(outgoing));

  strncpy(
    outgoing.nodeName,
    NODE_NAME,
    sizeof(outgoing.nodeName) - 1
  );

  // ==========================================================
  // STARTUP INFORMATION
  // ==========================================================

  Serial.println();
  Serial.println("================================");
  Serial.println("     TRINETRA TRACK NODE");
  Serial.println("================================");

  Serial.print("Node Name : ");
  Serial.println(NODE_NAME);

  Serial.print("MAC       : ");
  Serial.println(WiFi.macAddress());

  Serial.print("Channel   : ");
  Serial.println(WIFI_CHANNEL);

  Serial.println("Status    : ACTIVE");
  Serial.println("================================");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  esp_err_t result = esp_now_send(
    broadcastAddress,
    (uint8_t*)&outgoing,
    sizeof(outgoing)
  );

  if (result != ESP_OK) {
    Serial.print("[");
    Serial.print(NODE_NAME);
    Serial.println("] Send error");
  }

  // Broadcast once every second
  delay(1000);
}