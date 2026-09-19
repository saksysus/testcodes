#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

const char* NODE_NAME = "NODE2";

#define WIFI_CHANNEL 1

typedef struct {
  char nodeName[16];
} BeaconPacket;

BeaconPacket outgoing;

uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

void OnDataSent(
  const wifi_tx_info_t *tx_info,
  esp_now_send_status_t status
) {
  Serial.print("[");
  Serial.print(NODE_NAME);
  Serial.print("] Beacon: ");

  Serial.println(
    status == ESP_NOW_SEND_SUCCESS
      ? "SENT"
      : "FAILED"
  );
}

void setup() {

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    broadcastAddress,
    6
  );

  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("FAILED TO ADD PEER");
    return;
  }

  memset(&outgoing, 0, sizeof(outgoing));

  strncpy(
    outgoing.nodeName,
    NODE_NAME,
    sizeof(outgoing.nodeName) - 1
  );

  Serial.println();
  Serial.println("==========================");
  Serial.println("TRINETRA TRACK NODE");
  Serial.println("==========================");

  Serial.print("NAME: ");
  Serial.println(NODE_NAME);

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("STATUS: ACTIVE");
  Serial.println("==========================");
}

void loop() {

  esp_now_send(
    broadcastAddress,
    (uint8_t*)&outgoing,
    sizeof(outgoing)
  );

  delay(1000);
}