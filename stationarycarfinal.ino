/*
   ============================================================
   TRINETRA — STATIONARY DUMPER
   STABLE LiDAR + ILI9341 TFT + ESP-NOW
   ============================================================

   PURPOSE
   -------
   Detect vehicle with TF-Luna / TF-Luna-style LiDAR.

   <= DETECTION_THRESHOLD_CM
        |
        +----> TFT VEHICLE DETECTED
        |
        +----> ESP-NOW "MOVE LEFT" to CAR 1
        |
        +----> repeat while vehicle remains close

   When vehicle leaves:
        |
        +----> stop MOVE LEFT
        |
        +----> TFT returns to monitoring

   ============================================================
   MOVING CAR
   ============================================================

   MAC:
       00:70:07:E1:BC:EC

   ESP-NOW channel:
       1

   ============================================================
   PACKET
   ============================================================

   struct:
       uint8_t messageType;
       char message[64];

   1 = Curve
   2 = Slope
   3 = MOVE LEFT
   4 = CAR1 heartbeat
   5 = stationary heartbeat

   ============================================================
   TFT — YOUR KNOWN-WORKING WIRING
   ============================================================

   TFT VCC   -> 3.3V
   TFT GND   -> GND
   TFT CS    -> GPIO15
   TFT DC    -> GPIO2
   TFT RESET -> GPIO4
   TFT MOSI  -> GPIO23
   TFT CLK   -> GPIO18
   TFT MISO  -> GPIO19
   TFT LED   -> 3.3V

   IMPORTANT:
   The 6-argument Adafruit_ILI9341 constructor below is
   intentionally retained because this is the configuration
   from your TFT-working sketch.

   ============================================================
   LiDAR
   ============================================================

   LiDAR VCC -> 5V / VIN
   LiDAR GND -> GND
   LiDAR TX  -> ESP32 GPIO16
   LiDAR RX  -> ESP32 GPIO17

   UART2:
       RX = GPIO16
       TX = GPIO17

   ============================================================
   STABILITY
   ============================================================

   - No pulseIn()
   - No blocking LiDAR measurement
   - TFT and LiDAR use separate buses
   - TFT is not continuously full-screen-redrawn
   - Full TFT redraw only when screen state changes
   - ESP-NOW command and heartbeat are serialized
   - MOVE LEFT has priority over heartbeat
   - Detection uses consecutive readings
   - Detection uses hysteresis
   - Weak LiDAR readings do not instantly clear an alert
   - LiDAR timeout prevents a stuck alert forever
   - Only CAR 1 is accepted as an ESP-NOW sender

   ============================================================
*/

#include <Arduino.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


// ============================================================
// ======================== SETTINGS ==========================
// ============================================================

// ------------------------------------------------------------
// CHANGE THIS VALUE TO CHANGE THE DETECTION DISTANCE
// ------------------------------------------------------------
//
// 20 = trigger at <= 20 cm
// 25 = trigger at <= 25 cm
// 30 = trigger at <= 30 cm
// 40 = trigger at <= 40 cm
//
#define DETECTION_THRESHOLD_CM       25

// ------------------------------------------------------------
// CLEAR DISTANCE
// ------------------------------------------------------------
//
// Because this is larger than the trigger distance, the system
// has hysteresis:
//
//     TRIGGER <= 40 cm
//     CLEAR   >= 32 cm
//
// This prevents the alert from rapidly switching on/off if the
// LiDAR fluctuates around 25 cm.
//
#define CLEAR_THRESHOLD_CM           32

// ------------------------------------------------------------
// LiDAR filtering
// ------------------------------------------------------------

// Number of valid close readings required.
#define DETECTION_CONFIRM_READINGS   4

// Number of valid clear readings required.
#define CLEAR_CONFIRM_READINGS       4

// Ignore very weak TF-Luna frames.
#define MIN_SIGNAL_STRENGTH          100

// If no valid strong LiDAR frame arrives for this long,
// clear the vehicle alert.
#define LIDAR_STALE_TIMEOUT_MS       900UL

// ------------------------------------------------------------
// ESP-NOW
// ------------------------------------------------------------

#define WIFI_CHANNEL                 1

// How frequently MOVE LEFT is resent while vehicle remains
// inside the detection zone.
#define COMMAND_REPEAT_MS            700UL

// Stationary node heartbeat interval.
#define NODE_HEARTBEAT_MS            1000UL

// How long until CAR1 is considered disconnected.
#define CAR_LINK_TIMEOUT_MS          3500UL

// ------------------------------------------------------------
// LiDAR UART
// ------------------------------------------------------------

#define LIDAR_BAUD                  115200

#define LIDAR_RX_PIN                16
#define LIDAR_TX_PIN                17

// Maximum bytes processed from UART in one loop.
#define MAX_LIDAR_BYTES_PER_LOOP    90

// ------------------------------------------------------------
// TFT
// ------------------------------------------------------------

#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4

#define TFT_MOSI 23
#define TFT_SCK  18
#define TFT_MISO 19

// ------------------------------------------------------------
// TFT refresh timing
// ------------------------------------------------------------
//
// Much slower than your old 15/25 FPS full-screen redraw.
// This is intentional.
//
// Less SPI traffic = more CPU/radio time for ESP-NOW.
//
#define TFT_NORMAL_UPDATE_MS 500UL
#define TFT_ALERT_UPDATE_MS  350UL

// ------------------------------------------------------------
// COMMAND
// ------------------------------------------------------------

#define DUMPER_COMMAND "MOVE LEFT"


// ============================================================
// ===================== MESSAGE TYPES ========================
// ============================================================

#define MSG_CURVE          1
#define MSG_SLOPE          2
#define MSG_DUMPER_CMD     3
#define MSG_CAR_HEARTBEAT  4
#define MSG_NODE_HEARTBEAT 5


// ============================================================
// ====================== SHARED PACKET ========================
// ============================================================

typedef struct
{
  uint8_t messageType;
  char message[64];
}
AlertPacket;


// ============================================================
// ====================== MOVING CAR MAC ======================
// ============================================================

uint8_t carMAC[6] =
{
  0x00,
  0x70,
  0x07,
  0xE1,
  0xBC,
  0xEC
};


// ============================================================
// ======================== HARDWARE ==========================
// ============================================================

HardwareSerial lidar(2);


// ============================================================
// ========================== TFT =============================
// ============================================================
//
// IMPORTANT:
//
// This is intentionally the SAME constructor used in your
// original TFT-working sketch:
//
//   Adafruit_ILI9341 tft(
//     TFT_CS,
//     TFT_DC,
//     23,
//     18,
//     TFT_RST,
//     19
//   );
//
// Do NOT add SPI.begin().
//
// ============================================================

Adafruit_ILI9341 tft(
  TFT_CS,
  TFT_DC,
  TFT_MOSI,
  TFT_SCK,
  TFT_RST,
  TFT_MISO
);


// ============================================================
// ===================== VEHICLE STATE ========================
// ============================================================

bool vehicleDetected = false;

uint8_t closeCount = 0;
uint8_t clearCount = 0;

// Latest valid strong LiDAR frame.
unsigned long lastValidLidarMs = 0;

// IMPORTANT:
// This variable was missing in the previous version.
// It records the last successful MOVE LEFT transmission.
unsigned long lastCommandSentMs = 0;

uint16_t currentDistance = 0;
uint16_t currentStrength = 0;

bool lidarValid = false;


// ============================================================
// ====================== CAR LINK STATE ======================
// ============================================================

volatile unsigned long lastCarHeartbeatMs = 0;
volatile int carRSSI = 0;


// ============================================================
// ===================== ESP-NOW TX STATE =====================
// ============================================================

enum TxKind : uint8_t
{
  TX_IDLE = 0,
  TX_COMMAND,
  TX_HEARTBEAT
};

volatile TxKind txKind = TX_IDLE;


// ============================================================
// ================= COMMAND RESULT STATE ====================
// ============================================================

enum CommandResult : uint8_t
{
  CMD_RESULT_NONE = 0,
  CMD_RESULT_SENDING,
  CMD_RESULT_DELIVERED,
  CMD_RESULT_FAILED
};

volatile CommandResult commandResult =
  CMD_RESULT_NONE;


// ============================================================
// ======================== TFT STATE =========================
// ============================================================

enum ScreenMode : uint8_t
{
  SCREEN_NORMAL = 0,
  SCREEN_ALERT
};

ScreenMode previousScreen =
  SCREEN_NORMAL;

unsigned long lastTftUpdateMs = 0;

bool alertIndicator = false;


// ============================================================
// ====================== LiDAR PARSER ========================
// ============================================================

uint8_t lidarBuffer[9];

uint8_t lidarIndex = 0;


// ============================================================
// ================= FUNCTION DECLARATIONS ====================
// ============================================================

void readLidar();

void handleLidarReading(
  uint16_t distance,
  uint16_t strength
);

void updateVehicleState();

bool carLinked();

bool sendMoveLeft();

bool sendHeartbeat();

void updateTFT();

void drawNormalScreen(
  bool fullRedraw
);

void drawAlertScreen(
  bool fullRedraw
);

void drawCornerBrackets(
  uint16_t color
);

void printCentered(
  const char *text,
  int y,
  uint8_t size
);

void OnDataSent(
  const wifi_tx_info_t *tx_info,
  esp_now_send_status_t status
);

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
);


// ============================================================
// ======================== CAR LINK ==========================
// ============================================================

bool carLinked()
{
  unsigned long lastSeen =
    lastCarHeartbeatMs;

  return
    lastSeen != 0 &&
    millis() - lastSeen <=
    CAR_LINK_TIMEOUT_MS;
}


// ============================================================
// ===================== SEND MOVE LEFT =======================
// ============================================================

bool sendMoveLeft()
{
  // Do not transmit while another packet is awaiting callback.
  if (
    txKind != TX_IDLE
  )
  {
    return false;
  }

  AlertPacket packet = {};

  packet.messageType =
    MSG_DUMPER_CMD;

  strlcpy(
    packet.message,
    DUMPER_COMMAND,
    sizeof(packet.message)
  );

  txKind =
    TX_COMMAND;

  commandResult =
    CMD_RESULT_SENDING;

  esp_err_t result =
    esp_now_send(
      carMAC,
      reinterpret_cast<uint8_t *>(&packet),
      sizeof(packet)
    );

  if (
    result != ESP_OK
  )
  {
    txKind =
      TX_IDLE;

    commandResult =
      CMD_RESULT_FAILED;

    Serial.print(
      "[ESP-NOW] MOVE LEFT send error: "
    );

    Serial.println(
      (int)result
    );

    return false;
  }

  Serial.println(
    "[ESP-NOW] MOVE LEFT TX"
  );

  return true;
}


// ============================================================
// ================== SEND STATIONARY HEARTBEAT ===============
// ============================================================

bool sendHeartbeat()
{
  /*
     MOVE LEFT has priority.

     If MOVE LEFT is currently in flight, simply leave the
     heartbeat for the next loop.
  */

  if (
    txKind != TX_IDLE
  )
  {
    return false;
  }

  AlertPacket packet = {};

  packet.messageType =
    MSG_NODE_HEARTBEAT;

  strlcpy(
    packet.message,
    "STATIONARY_ALIVE",
    sizeof(packet.message)
  );

  txKind =
    TX_HEARTBEAT;

  esp_err_t result =
    esp_now_send(
      carMAC,
      reinterpret_cast<uint8_t *>(&packet),
      sizeof(packet)
    );

  if (
    result != ESP_OK
  )
  {
    txKind =
      TX_IDLE;

    Serial.print(
      "[ESP-NOW] Heartbeat error: "
    );

    Serial.println(
      (int)result
    );

    return false;
  }

  return true;
}


// ============================================================
// ================= ESP-NOW SEND CALLBACK ====================
// ============================================================

void OnDataSent(
  const wifi_tx_info_t *tx_info,
  esp_now_send_status_t status
)
{
  (void)tx_info;

  /*
     Save the type BEFORE clearing it.
  */

  TxKind completed =
    txKind;

  txKind =
    TX_IDLE;

  if (
    completed ==
    TX_COMMAND
  )
  {
    if (
      status ==
      ESP_NOW_SEND_SUCCESS
    )
    {
      commandResult =
        CMD_RESULT_DELIVERED;

      Serial.println(
        "[ESP-NOW] MOVE LEFT DELIVERED"
      );
    }
    else
    {
      commandResult =
        CMD_RESULT_FAILED;

      Serial.println(
        "[ESP-NOW] MOVE LEFT FAILED"
      );
    }
  }
}


// ============================================================
// ================= ESP-NOW RECEIVE CALLBACK =================
// ============================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
)
{
  if (
    !info ||
    !data
  )
  {
    return;
  }

  // ----------------------------------------------------------
  // Only accept our moving car.
  // ----------------------------------------------------------

  if (
    memcmp(
      info->src_addr,
      carMAC,
      6
    ) != 0
  )
  {
    return;
  }

  if (
    len !=
    (int)sizeof(AlertPacket)
  )
  {
    return;
  }

  AlertPacket packet;

  memcpy(
    &packet,
    data,
    sizeof(packet)
  );

  packet.message[
    sizeof(packet.message) - 1
  ] = '\0';

  // ----------------------------------------------------------
  // Only process CAR1 heartbeat.
  // ----------------------------------------------------------

  if (
    packet.messageType !=
    MSG_CAR_HEARTBEAT
  )
  {
    return;
  }

  if (
    strcmp(
      packet.message,
      "CAR1"
    ) != 0
  )
  {
    return;
  }

  lastCarHeartbeatMs =
    millis();

  if (
    info->rx_ctrl
  )
  {
    carRSSI =
      info->rx_ctrl->rssi;
  }
}


// ============================================================
// ======================= LiDAR READER =======================
// ============================================================

void readLidar()
{
  uint16_t bytesProcessed = 0;

  while (
    lidar.available() &&
    bytesProcessed <
    MAX_LIDAR_BYTES_PER_LOOP
  )
  {
    bytesProcessed++;

    uint8_t b =
      lidar.read();

    // --------------------------------------------------------
    // First header byte
    // --------------------------------------------------------

    if (
      lidarIndex == 0
    )
    {
      if (
        b == 0x59
      )
      {
        lidarBuffer[0] =
          b;

        lidarIndex =
          1;
      }

      continue;
    }

    // --------------------------------------------------------
    // Second header byte
    // --------------------------------------------------------

    if (
      lidarIndex == 1
    )
    {
      if (
        b == 0x59
      )
      {
        lidarBuffer[1] =
          b;

        lidarIndex =
          2;
      }
      else
      {
        lidarIndex =
          0;
      }

      continue;
    }

    // --------------------------------------------------------
    // Remaining frame
    // --------------------------------------------------------

    lidarBuffer[lidarIndex++] =
      b;

    if (
      lidarIndex >= 9
    )
    {
      uint8_t checksum = 0;

      for (
        uint8_t i = 0;
        i < 8;
        i++
      )
      {
        checksum +=
          lidarBuffer[i];
      }

      if (
        checksum ==
        lidarBuffer[8]
      )
      {
        uint16_t distance =
          lidarBuffer[2] |
          ((uint16_t)lidarBuffer[3] << 8);

        uint16_t strength =
          lidarBuffer[4] |
          ((uint16_t)lidarBuffer[5] << 8);

        handleLidarReading(
          distance,
          strength
        );
      }

      lidarIndex =
        0;
    }
  }
}


// ============================================================
// =================== PROCESS LiDAR FRAME ====================
// ============================================================

void handleLidarReading(
  uint16_t distance,
  uint16_t strength
)
{
  // ----------------------------------------------------------
  // Weak frame
  // ----------------------------------------------------------
  //
  // DO NOT immediately clear the vehicle alert here.
  //
  // This prevents one noisy/weak frame from causing:
  //
  //    DETECTED -> CLEARED -> DETECTED
  //
  // ----------------------------------------------------------

  if (
    strength <
    MIN_SIGNAL_STRENGTH
  )
  {
    return;
  }

  currentDistance =
    distance;

  currentStrength =
    strength;

  lidarValid =
    true;

  lastValidLidarMs =
    millis();

  // ==========================================================
  // VEHICLE INSIDE TRIGGER RANGE
  // ==========================================================

  if (
    distance <=
    DETECTION_THRESHOLD_CM
  )
  {
    clearCount =
      0;

    if (
      closeCount <
      255
    )
    {
      closeCount++;
    }

    if (
      closeCount >=
      DETECTION_CONFIRM_READINGS
    )
    {
      if (
        !vehicleDetected
      )
      {
        vehicleDetected =
          true;

        /*
           IMPORTANT:
           Set this to zero so the first MOVE LEFT is allowed
           immediately.
        */
        lastCommandSentMs =
          0;

        commandResult =
          CMD_RESULT_NONE;

        Serial.println();
        Serial.println(
          "========================================"
        );

        Serial.println(
          "[DUMPER] VEHICLE DETECTED"
        );

        Serial.print(
          "[DUMPER] Distance: "
        );

        Serial.print(
          distance
        );

        Serial.println(
          " cm"
        );

        Serial.println(
          "[DUMPER] MOVE LEFT ACTIVE"
        );

        Serial.println(
          "========================================"
        );
      }
    }

    return;
  }

  // ==========================================================
  // VEHICLE OUTSIDE TRIGGER RANGE
  // ==========================================================

  closeCount =
    0;

  // Nothing else to do if no vehicle is active.
  if (
    !vehicleDetected
  )
  {
    return;
  }

  /*
     HYSTERESIS:

     Trigger at <= 25 cm.

     Do not clear at 26 cm.

     Begin clear filtering only at >= 32 cm.
  */

  if (
    distance >=
    CLEAR_THRESHOLD_CM
  )
  {
    if (
      clearCount <
      255
    )
    {
      clearCount++;
    }
  }
  else
  {
    clearCount =
      0;
  }

  // ----------------------------------------------------------
  // Clear only after enough confirmed readings.
  // ----------------------------------------------------------

  if (
    clearCount >=
    CLEAR_CONFIRM_READINGS
  )
  {
    vehicleDetected =
      false;

    clearCount =
      0;

    closeCount =
      0;

    lastCommandSentMs =
      0;

    commandResult =
      CMD_RESULT_NONE;

    Serial.println();
    Serial.println(
      "[DUMPER] VEHICLE CLEARED"
    );

    Serial.println(
      "[DUMPER] MOVE LEFT STOPPED"
    );
  }
}


// ============================================================
// ==================== VEHICLE STATE =========================
// ============================================================

void updateVehicleState()
{
  if (
    !vehicleDetected
  )
  {
    return;
  }

  /*
     Safety timeout:

     If the LiDAR stream stops producing valid strong readings,
     don't leave MOVE LEFT active forever.
  */

  if (
    lastValidLidarMs != 0 &&
    millis() -
    lastValidLidarMs >
    LIDAR_STALE_TIMEOUT_MS
  )
  {
    vehicleDetected =
      false;

    closeCount =
      0;

    clearCount =
      0;

    lastCommandSentMs =
      0;

    commandResult =
      CMD_RESULT_NONE;

    Serial.println(
      "[DUMPER] LiDAR STALE -> ALERT CLEARED"
    );
  }
}


// ============================================================
// ======================== TFT HELPERS =======================
// ============================================================

void printCentered(
  const char *text,
  int y,
  uint8_t size
)
{
  tft.setTextSize(
    size
  );

  int width =
    strlen(text) *
    6 *
    size;

  int x =
    (320 - width) / 2;

  if (
    x < 0
  )
  {
    x = 0;
  }

  tft.setCursor(
    x,
    y
  );

  tft.print(
    text
  );
}


// ============================================================

void drawCornerBrackets(
  uint16_t color
)
{
  const int len =
    18;

  tft.drawLine(
    3, 3,
    3 + len, 3,
    color
  );

  tft.drawLine(
    3, 3,
    3, 3 + len,
    color
  );

  tft.drawLine(
    316, 3,
    316 - len, 3,
    color
  );

  tft.drawLine(
    316, 3,
    316, 3 + len,
    color
  );

  tft.drawLine(
    3, 236,
    3 + len, 236,
    color
  );

  tft.drawLine(
    3, 236,
    3, 236 - len,
    color
  );

  tft.drawLine(
    316, 236,
    316 - len, 236,
    color
  );

  tft.drawLine(
    316, 236,
    316, 236 - len,
    color
  );
}


// ============================================================
// ==================== NORMAL TFT SCREEN =====================
// ============================================================

void drawNormalScreen(
  bool fullRedraw
)
{
  if (
    fullRedraw
  )
  {
    tft.fillScreen(
      ILI9341_BLACK
    );

    drawCornerBrackets(
      ILI9341_GREEN
    );
  }

  // ----------------------------------------------------------
  // Header
  // ----------------------------------------------------------

  tft.fillRect(
    8, 8,
    304, 24,
    ILI9341_BLACK
  );

  tft.setTextColor(
    ILI9341_CYAN
  );

  tft.setTextSize(2);

  tft.setCursor(
    14,
    12
  );

  tft.print(
    "TRINETRA"
  );

  tft.setTextColor(
    ILI9341_GREEN
  );

  tft.setCursor(
    232,
    12
  );

  tft.print(
    "CLEAR"
  );

  // ----------------------------------------------------------
  // Distance
  // ----------------------------------------------------------

  tft.fillRect(
    15, 42,
    290, 60,
    ILI9341_BLACK
  );

  tft.setTextColor(
    ILI9341_WHITE
  );

  tft.setTextSize(4);

  tft.setCursor(
    20,
    50
  );

  if (
    lidarValid
  )
  {
    tft.print(
      currentDistance
    );
  }
  else
  {
    tft.print(
      "---"
    );
  }

  tft.setTextSize(2);

  tft.print(
    " cm"
  );

  // ----------------------------------------------------------
  // Thresholds
  // ----------------------------------------------------------

  tft.fillRect(
    15, 106,
    290, 18,
    ILI9341_BLACK
  );

  tft.setTextColor(
    ILI9341_YELLOW
  );

  tft.setTextSize(1);

  tft.setCursor(
    15,
    110
  );

  tft.print(
    "TRIGGER <= "
  );

  tft.print(
    DETECTION_THRESHOLD_CM
  );

  tft.print(
    " cm"
  );

  tft.setCursor(
    150,
    110
  );

  tft.print(
    "CLEAR >= "
  );

  tft.print(
    CLEAR_THRESHOLD_CM
  );

  tft.print(
    " cm"
  );

  // ----------------------------------------------------------
  // Signal
  // ----------------------------------------------------------

  tft.fillRect(
    15, 128,
    290, 18,
    ILI9341_BLACK
  );

  tft.setTextColor(
    ILI9341_WHITE
  );

  tft.setCursor(
    15,
    132
  );

  tft.print(
    "SIGNAL: "
  );

  if (
    lidarValid
  )
  {
    tft.print(
      currentStrength
    );
  }
  else
  {
    tft.print(
      "---"
    );
  }

  // ----------------------------------------------------------
  // Car link
  // ----------------------------------------------------------

  tft.fillRect(
    15, 150,
    290, 20,
    ILI9341_BLACK
  );

  bool linked =
    carLinked();

  tft.setTextColor(
    linked
      ? ILI9341_CYAN
      : ILI9341_YELLOW
  );

  tft.setTextSize(1);

  tft.setCursor(
    15,
    154
  );

  if (
    linked
  )
  {
    tft.print(
      "CAR LINK: CONNECTED"
    );

    tft.setCursor(
      180,
      154
    );

    tft.print(
      carRSSI
    );

    tft.print(
      " dBm"
    );
  }
  else
  {
    tft.print(
      "CAR LINK: SEARCHING"
    );
  }

  // ----------------------------------------------------------
  // Bottom status
  // ----------------------------------------------------------

  tft.fillRect(
    15, 184,
    290, 42,
    ILI9341_BLACK
  );

  tft.setTextColor(
    ILI9341_GREEN
  );

  tft.setTextSize(2);

  tft.setCursor(
    20,
    196
  );

  tft.print(
    "MONITORING"
  );
}


// ============================================================
// ===================== ALERT TFT SCREEN =====================
// ============================================================

void drawAlertScreen(
  bool fullRedraw
)
{
  // ----------------------------------------------------------
  // Full redraw ONLY when first entering alert.
  // ----------------------------------------------------------

  if (
    fullRedraw
  )
  {
    tft.fillScreen(
      ILI9341_BLACK
    );

    drawCornerBrackets(
      ILI9341_RED
    );

    // Warning triangle
    int cx = 160;
    int cy = 55;

    tft.fillTriangle(
      cx,
      cy - 28,
      cx - 30,
      cy + 22,
      cx + 30,
      cy + 22,
      ILI9341_YELLOW
    );

    tft.setTextColor(
      ILI9341_BLACK
    );

    tft.setTextSize(3);

    tft.setCursor(
      152,
      44
    );

    tft.print(
      "!"
    );

    // Main alert
    tft.setTextColor(
      ILI9341_RED
    );

    printCentered(
      "VEHICLE",
      95,
      3
    );

    printCentered(
      "DETECTED",
      125,
      3
    );

    // Command
    tft.setTextColor(
      ILI9341_WHITE
    );

    printCentered(
      "MOVE LEFT",
      165,
      2
    );
  }

  // ----------------------------------------------------------
  // Small flashing bar.
  //
  // We do NOT flash the entire TFT.
  // ----------------------------------------------------------

  alertIndicator =
    !alertIndicator;

  uint16_t indicatorColor =
    alertIndicator
      ? ILI9341_RED
      : ILI9341_YELLOW;

  tft.fillRect(
    10,
    7,
    300,
    5,
    indicatorColor
  );

  // ----------------------------------------------------------
  // Distance
  // ----------------------------------------------------------

  tft.fillRect(
    60, 196,
    200, 25,
    ILI9341_BLACK
  );

  char distanceText[40];

  snprintf(
    distanceText,
    sizeof(distanceText),
    "DISTANCE: %u CM",
    currentDistance
  );

  tft.setTextColor(
    ILI9341_WHITE
  );

  printCentered(
    distanceText,
    199,
    1
  );
}


// ============================================================
// ======================= TFT UPDATE =========================
// ============================================================

void updateTFT()
{
  unsigned long now =
    millis();

  ScreenMode desiredScreen =
    vehicleDetected
      ? SCREEN_ALERT
      : SCREEN_NORMAL;

  // ----------------------------------------------------------
  // Screen state changed.
  //
  // One full redraw only.
  // ----------------------------------------------------------

  if (
    desiredScreen !=
    previousScreen
  )
  {
    previousScreen =
      desiredScreen;

    lastTftUpdateMs =
      0;

    if (
      desiredScreen ==
      SCREEN_ALERT
    )
    {
      drawAlertScreen(
        true
      );
    }
    else
    {
      drawNormalScreen(
        true
      );
    }

    lastTftUpdateMs =
      now;

    return;
  }

  // ----------------------------------------------------------
  // Normal display
  // ----------------------------------------------------------

  if (
    desiredScreen ==
    SCREEN_NORMAL
  )
  {
    if (
      now -
      lastTftUpdateMs >=
      TFT_NORMAL_UPDATE_MS
    )
    {
      lastTftUpdateMs =
        now;

      drawNormalScreen(
        false
      );
    }

    return;
  }

  // ----------------------------------------------------------
  // Alert display
  // ----------------------------------------------------------

  if (
    now -
    lastTftUpdateMs >=
    TFT_ALERT_UPDATE_MS
  )
  {
    lastTftUpdateMs =
      now;

    drawAlertScreen(
      false
    );
  }
}


// ============================================================
// ============================ SETUP ==========================
// ============================================================

void setup()
{
  Serial.begin(
    115200
  );

  delay(
    500
  );

  Serial.println();
  Serial.println(
    "===================================================="
  );

  Serial.println(
    " TRINETRA STATIONARY DUMPER"
  );

  Serial.println(
    " STABLE LiDAR + TFT + ESP-NOW"
  );

  Serial.println(
    "===================================================="
  );

  // ==========================================================
  // LiDAR UART2
  // ==========================================================

  lidar.begin(
    LIDAR_BAUD,
    SERIAL_8N1,
    LIDAR_RX_PIN,
    LIDAR_TX_PIN
  );

  Serial.println(
    "[LIDAR] UART2 GPIO16/17"
  );

  // ==========================================================
  // TFT
  // ==========================================================

  /*
     IMPORTANT:
     Do exactly what the working TFT sketch did.

     No SPI.begin().
     No alternate hardware SPI constructor.
  */

  tft.begin();

  tft.setRotation(
    1
  );

  tft.fillScreen(
    ILI9341_BLACK
  );

  tft.setTextColor(
    ILI9341_CYAN
  );

  tft.setTextSize(2);

  tft.setCursor(
    20,
    95
  );

  tft.println(
    "TRINETRA"
  );

  tft.setTextSize(1);

  tft.setCursor(
    20,
    125
  );

  tft.println(
    "STATIONARY DUMPER"
  );

  delay(
    600
  );

  // ==========================================================
  // WIFI
  // ==========================================================

  WiFi.mode(
    WIFI_STA
  );

  WiFi.setSleep(
    false
  );

  esp_wifi_set_ps(
    WIFI_PS_NONE
  );

  WiFi.disconnect();

  delay(
    100
  );

  // Force channel 1.
  esp_err_t channelResult =
    esp_wifi_set_channel(
      WIFI_CHANNEL,
      WIFI_SECOND_CHAN_NONE
    );

  if (
    channelResult !=
    ESP_OK
  )
  {
    Serial.print(
      "[WiFi] Channel error: "
    );

    Serial.println(
      (int)channelResult
    );
  }

  // ==========================================================
  // ESP-NOW
  // ==========================================================

  if (
    esp_now_init() !=
    ESP_OK
  )
  {
    Serial.println(
      "[ESP-NOW] INIT FAILED"
    );

    delay(
      1000
    );

    ESP.restart();
  }

  esp_now_register_send_cb(
    OnDataSent
  );

  esp_now_register_recv_cb(
    OnDataRecv
  );

  // ==========================================================
  // ADD ONLY MOVING CAR
  // ==========================================================

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    carMAC,
    6
  );

  peerInfo.channel =
    WIFI_CHANNEL;

  peerInfo.encrypt =
    false;

  peerInfo.ifidx =
    WIFI_IF_STA;

  esp_err_t peerResult =
    esp_now_add_peer(
      &peerInfo
    );

  if (
    peerResult !=
    ESP_OK &&
    peerResult !=
    ESP_ERR_ESPNOW_EXIST
  )
  {
    Serial.print(
      "[ESP-NOW] Peer error: "
    );

    Serial.println(
      (int)peerResult
    );
  }

  // ==========================================================
  // STARTUP INFORMATION
  // ==========================================================

  Serial.println();

  Serial.print(
    "Stationary MAC : "
  );

  Serial.println(
    WiFi.macAddress()
  );

  Serial.println(
    "Car MAC        : 00:70:07:E1:BC:EC"
  );

  Serial.print(
    "Channel        : "
  );

  Serial.println(
    WIFI_CHANNEL
  );

  Serial.print(
    "Trigger        : <= "
  );

  Serial.print(
    DETECTION_THRESHOLD_CM
  );

  Serial.println(
    " cm"
  );

  Serial.print(
    "Clear          : >= "
  );

  Serial.print(
    CLEAR_THRESHOLD_CM
  );

  Serial.println(
    " cm"
  );

  Serial.print(
    "Detect filter  : "
  );

  Serial.print(
    DETECTION_CONFIRM_READINGS
  );

  Serial.println(
    " readings"
  );

  Serial.print(
    "Clear filter   : "
  );

  Serial.print(
    CLEAR_CONFIRM_READINGS
  );

  Serial.println(
    " readings"
  );

  Serial.print(
    "MOVE LEFT      : every "
  );

  Serial.print(
    COMMAND_REPEAT_MS
  );

  Serial.println(
    " ms"
  );

  Serial.println();

  Serial.println(
    "TFT            : SOFTWARE SPI"
  );

  Serial.println(
    "LiDAR          : UART2"
  );

  Serial.println(
    "ESP-NOW        : CHANNEL 1"
  );

  Serial.println(
    "Destination    : CAR 1 only"
  );

  Serial.println();

  Serial.println(
    "SYSTEM READY"
  );

  Serial.println(
    "===================================================="
  );

  // ==========================================================
  // Initial TFT
  // ==========================================================

  previousScreen =
    SCREEN_NORMAL;

  lastTftUpdateMs =
    0;

  updateTFT();
}


// ============================================================
// ============================= LOOP ==========================
// ============================================================

void loop()
{
  unsigned long now =
    millis();

  // ==========================================================
  // 1. READ LiDAR
  // ==========================================================

  readLidar();

  // ==========================================================
  // 2. UPDATE VEHICLE STATE
  // ==========================================================

  updateVehicleState();

  // ==========================================================
  // 3. SEND MOVE LEFT
  // ==========================================================

  if (
    vehicleDetected
  )
  {
    if (
      lastCommandSentMs == 0 ||
      now -
      lastCommandSentMs >=
      COMMAND_REPEAT_MS
    )
    {
      if (
        sendMoveLeft()
      )
      {
        /*
           This is the missing variable that caused your
           compilation error in the previous version.
        */

        lastCommandSentMs =
          now;
      }
    }
  }
  else
  {
    lastCommandSentMs =
      0;
  }

  // ==========================================================
  // 4. SEND STATIONARY HEARTBEAT
  // ==========================================================

  static unsigned long
    lastHeartbeatMs = 0;

  if (
    now -
    lastHeartbeatMs >=
    NODE_HEARTBEAT_MS
  )
  {
    /*
       If MOVE LEFT is currently in flight, sendHeartbeat()
       returns false and the heartbeat is retried on the next
       loop instead of interfering with the command.
    */

    if (
      sendHeartbeat()
    )
    {
      lastHeartbeatMs =
        now;
    }
  }

  // ==========================================================
  // 5. UPDATE TFT
  // ==========================================================

  updateTFT();

  // ==========================================================
  // 6. VERY SMALL YIELD
  // ==========================================================

  delay(
    2
  );
}
