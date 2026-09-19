/*
   TRINETRA — STATIONARY DUMPER (LiDAR/ToF + TFT)  ->  talks ONLY to the moving car
   ============================================================
   What this board does
   - Watches the front with a TF-Luna style LiDAR (the "ToF" sensor).
   - When a vehicle comes closer than CRITICAL_DISTANCE_CM it sends the
     command DUMPER_COMMAND ("MOVE LEFT") straight to the car's MAC.
   - The same command is shown on this TFT (flashing alert screen, plus
     whether the car actually received it).
   - The car's web page shows the same command (see car_node.ino).

   "Only connects to the moving one"
   - The only ESP-NOW peer added is the car (carMac). No broadcast peer,
     no console board.
   - Every incoming packet whose sender is not the car is dropped.
   - The car broadcasts a 1 Hz heartbeat; that's how this board knows the
     car is in range ("CAR LINK: CONNECTED" on the TFT).

   Trigger
   - LiDAR distance < CRITICAL_DISTANCE_CM for TRIGGER_CONFIRM_READINGS
     readings in a row (filters single noisy frames).
   - Optional: set REQUIRE_CAR_SIGNAL_CONFIRM to 1 and the car's heartbeat
     must ALSO be strong (RSSI_NEAR_THRESHOLD_DBM) before the command goes
     out — the original two-sensor fusion.

   Packet format is shared by every node (track node, this board, car):
     { uint8_t messageType; char message[64]; }
     1 = Curve, 2 = Slope (track node)   3 = dumper command   4 = car heartbeat

   TFT CONNECTION (unchanged wiring)
   TFT VCC   -> 3.3V   | TFT GND -> GND   | TFT CS  -> GPIO15
   TFT DC    -> GPIO2  | TFT RESET -> GPIO4
   TFT MOSI  -> GPIO23 | TFT CLK -> GPIO18 | TFT MISO -> GPIO19
   TFT LED   -> 3.3V

   LiDAR CONNECTION
   LiDAR VCC -> 5V/VIN | LiDAR GND -> GND
   LiDAR TX  -> ESP32 GPIO16 | LiDAR RX -> ESP32 GPIO17

   LIBRARIES: Adafruit GFX Library, Adafruit ILI9341
   (WiFi / esp_now / esp_wifi ship with arduino-esp32 core 3.x)

   LiDAR UART: 115200 baud, 9-byte TF-Luna style frame
   ============================================================
*/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================
// TFT
// ============================================================
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST   4

// Hardware SPI — your wires (SCK 18, MISO 19, MOSI 23) are already the
// ESP32's default SPI pins, so this is a drop-in and MUCH faster than the
// software-SPI constructor (full-screen redraws no longer stall the loop).
Adafruit_ILI9341 tft(
  TFT_CS,
  TFT_DC,
  23,  // MOSI
  18,  // SCK
  TFT_RST,
  19   // MISO
);
// Old software-SPI version, if you ever need to go back:
// Adafruit_ILI9341 tft(TFT_CS, TFT_DC, 23, 18, TFT_RST, 19);

// ============================================================
// LiDAR UART PINS
// ============================================================
#define RXD2 16
#define TXD2 17
HardwareSerial tf(2);

// ============================================================
// ================= USER SETTINGS ============================
// ============================================================
#define TFT_SAFE_FPS      5
#define TFT_WARNING_FPS  15
#define TFT_CRITICAL_FPS 25

#define CRITICAL_DISTANCE_CM     50    // vehicle this close -> command goes out
#define WARNING_DISTANCE_CM     150
#define MIN_SIGNAL_STRENGTH     100
#define TRIGGER_CONFIRM_READINGS  5    // consecutive close readings required

#define WIFI_CHANNEL                1  // must match the car (car_node.ino)
#define NEAR_HOLD_MS              800  // how long a "close" reading counts as current
#define ALERT_TIMEOUT_MS         3000  // clear the alert this long after the vehicle is gone
#define COMMAND_RESEND_INTERVAL_MS 1000 // repeat the command while the vehicle stays close
#define CAR_LINK_TIMEOUT_MS      3000  // no heartbeat for this long = link lost

#define REQUIRE_CAR_SIGNAL_CONFIRM  0  // 1 = also need a strong car heartbeat (LiDAR + RSSI fusion)
#define RSSI_NEAR_THRESHOLD_DBM   -50  // only used when the line above is 1

#define DUMPER_COMMAND "MOVE LEFT"     // what the car is told to do

// ============================================================
// SHARED PACKET (identical on track node, this board and the car)
// ============================================================
#define MSG_CURVE          1
#define MSG_SLOPE          2
#define MSG_DUMPER_CMD     3
#define MSG_CAR_HEARTBEAT  4

typedef struct {
  uint8_t messageType;
  char message[64];
} AlertPacket;

// The moving car (its Wi-Fi STA MAC — the car prints it at boot)
uint8_t carMac[6] = {0x00, 0x70, 0x07, 0xE1, 0xBC, 0xEC};

// ============================================================
// STATE
// ============================================================
enum CmdState : uint8_t { CMD_IDLE, CMD_SENDING, CMD_DELIVERED, CMD_NO_REPLY };

// written from the Wi-Fi task callbacks, read in loop()
volatile CmdState     cmdState = CMD_IDLE;
volatile unsigned long lastCarSeen = 0;
volatile int          carRssi = 0;
volatile unsigned long lastNearSignalTime = 0;

unsigned long lastCriticalLidarTime = 0;   // 0 = never
unsigned long lastCommandSent = 0;
unsigned long dumperAlertLastSeen = 0;
bool dumperAlertActive = false;
bool tftMode_alert = false;
uint8_t nearCount = 0;

// LiDAR / display
uint16_t currentDistance = 0;
uint16_t currentStrength = 0;
bool lidarValid = false;

unsigned long lastDisplayUpdate = 0;
unsigned long lidarRateTimer = 0;
unsigned long lidarRateCount = 0;
float actualLidarHz = 0.0;

enum Status { STATUS_SAFE, STATUS_WARNING, STATUS_CRITICAL };
Status currentStatus = STATUS_SAFE;
Status lastDisplayedStatus = STATUS_SAFE;
bool forceFullRedraw = true;

const char* NODE_NAME = "TRINETRA";

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
void readLidar();
void handleReading(uint16_t distance, uint16_t strength);
bool carLinked();
void updateDisplay();
void drawDumperAlertScreen();
void drawCornerBrackets(uint16_t color);
void drawHeader(uint16_t accent, const char* label);
void drawProximityBar();
void printCentered(const char* s, int y, uint8_t size);
unsigned long getDisplayInterval(Status status);
void sendCommandToCar(const char* cmd);
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);

// ============================================================
// SETUP
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== TRINETRA STATIONARY DUMPER ===");

  tf.begin(115200, SERIAL_8N1, RXD2, TXD2);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(20, 100);
  tft.println("TRINETRA BOOTING...");
  delay(600);
  tft.fillScreen(ILI9341_BLACK);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, restarting...");
    delay(1000);
    ESP.restart();
  }

  // NOTE: if your core is older than 3.3 and this line won't compile, change the
  // callback signature to: void OnDataSent(const uint8_t *mac, esp_now_send_status_t status)
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // The car is the ONLY peer this board ever talks to.
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, carMac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add the car as a peer");
  }

  Serial.print("This board MAC : ");
  Serial.println(WiFi.macAddress());
  Serial.println("Talking only to: 00:70:07:E1:BC:EC (moving car)");
  Serial.print("Command on trigger: ");
  Serial.println(DUMPER_COMMAND);

  lidarRateTimer = millis();
  Serial.println("System ready.");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop()
{
  unsigned long now = millis();

  readLidar();

  if (now - lidarRateTimer >= 1000) {
    actualLidarHz = lidarRateCount;
    lidarRateCount = 0;
    lidarRateTimer = now;
  }

  // ---- car link status (Serial only, the TFT shows it on the dashboard) ----
  static bool wasLinked = false;
  bool linked = carLinked();
  if (linked != wasLinked) {
    wasLinked = linked;
    Serial.println(linked ? "Car link: CONNECTED" : "Car link: LOST");
  }

  // ---- is a vehicle close right now? ----
  bool lidarRecent = (lastCriticalLidarTime != 0) && (now - lastCriticalLidarTime) <= NEAR_HOLD_MS;
#if REQUIRE_CAR_SIGNAL_CONFIRM
  bool signalRecent = (lastNearSignalTime != 0) && (now - lastNearSignalTime) <= NEAR_HOLD_MS;
  bool vehicleNear = lidarRecent && signalRecent;
#else
  bool vehicleNear = lidarRecent;
#endif

  if (vehicleNear) {
    bool firstTrigger = !dumperAlertActive;
    dumperAlertActive = true;
    dumperAlertLastSeen = now;

    if (firstTrigger || now - lastCommandSent >= COMMAND_RESEND_INTERVAL_MS) {
      lastCommandSent = now;
      if (firstTrigger) {
        Serial.print("VEHICLE DETECTED at ");
        Serial.print(currentDistance);
        Serial.println(" cm");
      }
      Serial.print("SENDING CMD: ");
      Serial.println(DUMPER_COMMAND);
      sendCommandToCar(DUMPER_COMMAND);
    }
  }

  if (dumperAlertActive && now - dumperAlertLastSeen > ALERT_TIMEOUT_MS) {
    dumperAlertActive = false;
    forceFullRedraw = true;
    cmdState = CMD_IDLE;
    Serial.println("Vehicle cleared.");
  }

  // ---- dashboard update ----
  unsigned long displayInterval = dumperAlertActive ? 300 : getDisplayInterval(currentStatus);

  if (now - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = now;

    if (dumperAlertActive) {
      drawDumperAlertScreen();
    } else {
      // Always show the dashboard, even if LiDAR has not produced a valid frame yet.
      updateDisplay();
    }
  }
}

// ============================================================
// LiDAR FRAME PARSER (never blocks)
// ============================================================
void readLidar()
{
  static uint8_t buffer[9];
  static uint8_t index = 0;

  while (tf.available())
  {
    uint8_t b = tf.read();
    if (index == 0) {
      if (b == 0x59) buffer[index++] = b;
    } else if (index == 1) {
      if (b == 0x59) buffer[index++] = b;
      else index = 0;
    } else {
      buffer[index++] = b;
      if (index == 9) {
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < 8; i++) checksum += buffer[i];
        if (checksum == buffer[8]) {
          uint16_t distance = buffer[2] | ((uint16_t)buffer[3] << 8);
          uint16_t strength = buffer[4] | ((uint16_t)buffer[5] << 8);
          handleReading(distance, strength);
          lidarRateCount++;
        }
        index = 0;
      }
    }
  }
}

// ============================================================
// HANDLE LiDAR READING
// ============================================================
void handleReading(uint16_t distance, uint16_t strength)
{
  if (strength < MIN_SIGNAL_STRENGTH) {
    nearCount = 0;          // weak frame = no reliable target
    return;
  }

  currentDistance = distance;
  currentStrength = strength;
  lidarValid = true;

  if (distance < CRITICAL_DISTANCE_CM) {
    currentStatus = STATUS_CRITICAL;
    if (nearCount < 255) nearCount++;
    if (nearCount >= TRIGGER_CONFIRM_READINGS) {
      lastCriticalLidarTime = millis();
    }
  }
  else {
    nearCount = 0;
    currentStatus = (distance < WARNING_DISTANCE_CM) ? STATUS_WARNING : STATUS_SAFE;
  }
}

bool carLinked()
{
  unsigned long seen = lastCarSeen;
  return seen != 0 && (millis() - seen) <= CAR_LINK_TIMEOUT_MS;
}

// ============================================================
// DISPLAY INTERVAL
// ============================================================
unsigned long getDisplayInterval(Status status)
{
  uint16_t fps = (status == STATUS_CRITICAL) ? TFT_CRITICAL_FPS :
                 (status == STATUS_WARNING) ? TFT_WARNING_FPS : TFT_SAFE_FPS;
  return fps == 0 ? 1000 : (1000UL / fps);
}

// ============================================================
// HUD HELPERS
// ============================================================
void printCentered(const char* s, int y, uint8_t size)
{
  tft.setTextSize(size);
  int w = strlen(s) * 6 * size;
  tft.setCursor((320 - w) / 2, y);
  tft.print(s);
}

void drawCornerBrackets(uint16_t color)
{
  const int len = 18;
  tft.drawLine(3, 3, 3 + len, 3, color);
  tft.drawLine(3, 3, 3, 3 + len, color);
  tft.drawLine(316, 3, 316 - len, 3, color);
  tft.drawLine(316, 3, 316, 3 + len, color);
  tft.drawLine(3, 236, 3 + len, 236, color);
  tft.drawLine(3, 236, 3, 236 - len, color);
  tft.drawLine(316, 236, 316 - len, 236, color);
  tft.drawLine(316, 236, 316, 236 - len, color);
}

void drawHeader(uint16_t accent, const char* label)
{
  tft.drawRect(6, 6, 308, 26, accent);
  tft.setTextColor(accent);
  tft.setTextSize(2);
  tft.setCursor(14, 12);
  tft.print(NODE_NAME);
  tft.setCursor(320 - 12 - (strlen(label) * 12), 12);
  tft.print(label);
}

void drawProximityBar()
{
  int barX = 15, barY = 150, barW = 290, barH = 20;
  int maxRange = 300;
  int clamped = constrain((int)currentDistance, 0, maxRange);
  int fillW = map(maxRange - clamped, 0, maxRange, 0, barW - 4);

  uint16_t barColor = (currentStatus == STATUS_CRITICAL) ? ILI9341_RED :
                      (currentStatus == STATUS_WARNING) ? ILI9341_YELLOW : ILI9341_GREEN;

  tft.drawRect(barX, barY, barW, barH, ILI9341_WHITE);
  tft.fillRect(barX + 2, barY + 2, fillW, barH - 4, barColor);
}

// ============================================================
// NORMAL DASHBOARD
// ============================================================
void updateDisplay()
{
  if (tftMode_alert) { tftMode_alert = false; forceFullRedraw = true; }

  uint16_t accent = (currentStatus == STATUS_CRITICAL) ? ILI9341_RED :
                    (currentStatus == STATUS_WARNING) ? ILI9341_YELLOW : ILI9341_GREEN;
  const char* label = (currentStatus == STATUS_CRITICAL) ? "DANGER" :
                      (currentStatus == STATUS_WARNING) ? "CAUTION" : "CLEAR";

  if (currentStatus != lastDisplayedStatus || forceFullRedraw) {
    tft.fillScreen(ILI9341_BLACK);
    drawCornerBrackets(accent);
    lastDisplayedStatus = currentStatus;
    forceFullRedraw = false;
  }

  drawHeader(accent, label);

  tft.fillRect(15, 40, 290, 100, ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(4);
  tft.setCursor(20, 45);
  if (lidarValid) {
    tft.print(currentDistance);
  } else {
    tft.print("---");
  }
  tft.setTextSize(2);
  tft.print(" cm");

  tft.setTextSize(1);
  tft.setCursor(20, 90);
  tft.print("LiDAR ");
  tft.print(actualLidarHz, 0);
  tft.print(" Hz   |   Refresh ");
  tft.print(getDisplayInterval(currentStatus) > 0 ? (1000 / getDisplayInterval(currentStatus)) : 0);
  tft.print(" fps");

  drawProximityBar();

  tft.fillRect(15, 180, 290, 50, ILI9341_BLACK);
  tft.setTextColor(accent);
  tft.setTextSize(3);
  tft.setCursor(20, 185);
  tft.print(label);

  bool linked = carLinked();
  tft.setTextColor(linked ? ILI9341_CYAN : ILI9341_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(15, 220);
  tft.print(linked ? "CAR LINK: CONNECTED" : "CAR LINK: SEARCHING...");
}

// ============================================================
// VEHICLE-DETECTED ALERT SCREEN (flashes, shows the command + delivery)
// ============================================================
void drawDumperAlertScreen()
{
  tftMode_alert = true;

  static bool flashOn = false;
  flashOn = !flashOn;

  uint16_t bg = flashOn ? ILI9341_RED : ILI9341_BLACK;
  uint16_t accent = flashOn ? ILI9341_WHITE : ILI9341_RED;

  tft.fillScreen(bg);
  drawCornerBrackets(accent);

  // warning triangle
  int cx = 160, cy = 62;
  tft.fillTriangle(cx, cy - 28, cx - 30, cy + 22, cx + 30, cy + 22, ILI9341_YELLOW);
  tft.fillTriangle(cx, cy - 16, cx - 18, cy + 14, cx + 18, cy + 14, bg);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(cx - 8, cy - 8);
  tft.print("!");

  tft.setTextColor(accent);
  printCentered("VEHICLE", 105, 3);
  printCentered("DETECTED", 135, 3);

  // the command that was sent to the car
  char line[40];
  snprintf(line, sizeof(line), "CMD: %s", DUMPER_COMMAND);
  tft.setTextColor(ILI9341_WHITE);
  printCentered(line, 178, 2);

  // did the car actually get it?
  CmdState st = cmdState;
  const char* status = (st == CMD_DELIVERED) ? "CAR RECEIVED" :
                       (st == CMD_NO_REPLY)  ? "NO REPLY - RETRY" : "SENDING...";
  printCentered(status, 204, 2);
}

// ============================================================
// ESP-NOW
// ============================================================
void sendCommandToCar(const char* cmd)
{
  AlertPacket pkt = {};
  pkt.messageType = MSG_DUMPER_CMD;
  strncpy(pkt.message, cmd, sizeof(pkt.message) - 1);

  cmdState = CMD_SENDING;
  esp_err_t result = esp_now_send(carMac, (uint8_t *)&pkt, sizeof(pkt));
  if (result != ESP_OK) {
    cmdState = CMD_NO_REPLY;
    Serial.print("Send error: ");
    Serial.println(result);
  }
}

// MAC-level result: SUCCESS means the car's radio acknowledged the packet.
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
  cmdState = (status == ESP_NOW_SEND_SUCCESS) ? CMD_DELIVERED : CMD_NO_REPLY;
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  // Only the moving car is allowed to talk to this board.
  if (memcmp(info->src_addr, carMac, 6) != 0) return;
  if (len != sizeof(AlertPacket)) return;
  if (data[0] != MSG_CAR_HEARTBEAT) return;

  lastCarSeen = millis();
  carRssi = info->rx_ctrl->rssi;
  if (carRssi >= RSSI_NEAR_THRESHOLD_DBM) {
    lastNearSignalTime = millis();
  }
}