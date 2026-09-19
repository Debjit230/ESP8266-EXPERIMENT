#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

extern "C" {
  #include "user_interface.h"
}

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define OLED_SDA     D3  // GPIO0
#define OLED_SCL     D4  // GPIO2
#define EXTERNAL_LED D1  // Blips on target detection

// Maximum tracked targets on the radar screen
#define MAX_TARGETS 8

struct Target {
  uint8_t mac[6];
  int rssi;
  float angle;      // Angle on radar sweep (radians)
  unsigned long lastSeen;
  bool active;
};

Target targets[MAX_TARGETS];

uint8_t currentChannel = 1;
unsigned long lastChannelHop = 0;
unsigned long lastRadarDraw = 0;
float sweepAngle = 0.0;

// Promiscuous RX packet structure for ESP8266 SDK
struct RxControl {
  signed rssi: 8;
  unsigned rate: 4;
  unsigned is_group: 1;
  unsigned: 1;
  unsigned sig_mode: 2;
  unsigned legacy_length: 12;
  unsigned damatch0: 1;
  unsigned damatch1: 1;
  unsigned bssidmatch0: 1;
  unsigned bssidmatch1: 1;
  unsigned MCS: 7;
  unsigned CWB: 1;
  unsigned HT_length: 16;
  unsigned Smoothing: 1;
  unsigned Not_Sounding: 1;
  unsigned: 1;
  unsigned Aggregation: 1;
  unsigned STBC: 2;
  unsigned FEC_CODING: 1;
  unsigned SGI: 1;
  unsigned rxend_state: 8;
  unsigned ampdu_cnt: 8;
  unsigned channel: 4;
  unsigned: 12;
};

struct SnifferPacket {
  struct RxControl rx_ctrl;
  uint8_t buf[112];
  uint16_t cnt;
  uint16_t len;
};

// Update or register a target in the radar list
void registerTarget(uint8_t* mac, int rssi) {
  int targetIndex = -1;

  for (int i = 0; i < MAX_TARGETS; i++) {
    if (targets[i].active && memcmp(targets[i].mac, mac, 6) == 0) {
      targetIndex = i;
      break;
    }
  }

  // If new device, allocate free or oldest slot
  if (targetIndex == -1) {
    for (int i = 0; i < MAX_TARGETS; i++) {
      if (!targets[i].active) {
        targetIndex = i;
        break;
      }
    }
  }

  if (targetIndex == -1) {
    targetIndex = random(0, MAX_TARGETS); // Replace random slot if full
  }

  memcpy(targets[targetIndex].mac, mac, 6);
  targets[targetIndex].rssi = rssi;
  targets[targetIndex].lastSeen = millis();

  // Assign fixed random bearing angle if new
  if (!targets[targetIndex].active) {
    targets[targetIndex].angle = random(0, 360) * (PI / 180.0);
  }
  targets[targetIndex].active = true;

  // Pulse LED on D1 for detection ping
  digitalWrite(EXTERNAL_LED, HIGH);
}

// Sniffer callback executed for every raw 802.11 air packet
void snifferCallback(uint8_t *buf, uint16_t len) {
  if (len == 12) {
    return; // Management frame header only
  }

  struct SnifferPacket *sniffer = (struct SnifferPacket*) buf;
  int rssi = sniffer->rx_ctrl.rssi;

  // Frame control byte: 0x40 is Probe Request
  uint8_t frameType = sniffer->buf[0];
  if (frameType == 0x40) {
    uint8_t *srcMac = &sniffer->buf[10]; // Client source MAC starts at byte 10
    registerTarget(srcMac, rssi);
  }
}

void setup() {
  pinMode(EXTERNAL_LED, OUTPUT);
  digitalWrite(EXTERNAL_LED, LOW);

  Serial.begin(115200);
  delay(200);

  // Initialize OLED on D3 & D4
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  // Setup Wi-Fi Promiscuous Mode
  WiFi.persistent(false);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  wifi_set_opmode(STATION_MODE);
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(snifferCallback);
  wifi_promiscuous_enable(1);

  Serial.println("Wi-Fi Radar Sniffer Online. Sweeping 2.4 GHz channels...");
}

void drawRadarScreen() {
  display.clearDisplay();

  const int centerX = 40;
  const int centerY = 32;
  const int maxRadius = 30;

  // 1. Draw Radar Scope Rings
  display.drawCircle(centerX, centerY, 10, SSD1306_WHITE);
  display.drawCircle(centerX, centerY, 20, SSD1306_WHITE);
  display.drawCircle(centerX, centerY, maxRadius, SSD1306_WHITE);

  // Crosshairs
  display.drawLine(centerX - maxRadius, centerY, centerX + maxRadius, centerY, SSD1306_WHITE);
  display.drawLine(centerX, centerY - maxRadius, centerX, centerY + maxRadius, SSD1306_WHITE);

  // 2. Draw Rotating Sweep Line
  int sweepX = centerX + cos(sweepAngle) * maxRadius;
  int sweepY = centerY + sin(sweepAngle) * maxRadius;
  display.drawLine(centerX, centerY, sweepX, sweepY, SSD1306_WHITE);

  // 3. Draw Detected Target "Blips"
  int activeCount = 0;
  int closestRSSI = -100;

  for (int i = 0; i < MAX_TARGETS; i++) {
    if (targets[i].active) {
      if (millis() - targets[i].lastSeen > 6000) {
        targets[i].active = false; // Expire inactive targets after 6 seconds
        continue;
      }

      activeCount++;
      if (targets[i].rssi > closestRSSI) {
        closestRSSI = targets[i].rssi;
      }

      // Map RSSI (-95 dBm to -35 dBm) to radar radius (30 to 4 px)
      int distanceRadius = map(targets[i].rssi, -95, -35, maxRadius - 2, 4);
      distanceRadius = constrain(distanceRadius, 4, maxRadius - 2);

      int targetX = centerX + cos(targets[i].angle) * distanceRadius;
      int targetY = centerY + sin(targets[i].angle) * distanceRadius;

      // Draw blip (solid small square)
      display.fillRect(targetX - 1, targetY - 1, 3, 3, SSD1306_WHITE);
    }
  }

  // 4. Radar Info Sidebar
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(76, 4);
  display.print("RADAR");

  display.setCursor(76, 18);
  display.printf("CH: %2d", currentChannel);

  display.setCursor(76, 32);
  display.printf("TGT: %d", activeCount);

  display.setCursor(76, 46);
  if (activeCount > 0) {
    display.printf("%ddBm", closestRSSI);
  } else {
    display.print("SCAN");
  }

  display.display();
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Channel Hopping (every 180ms to scan all 13 channels)
  if (currentMillis - lastChannelHop >= 180) {
    lastChannelHop = currentMillis;
    currentChannel++;
    if (currentChannel > 13) currentChannel = 1;
    wifi_set_channel(currentChannel);
  }

  // 2. Refresh Radar Display (~20 FPS) & Advance Sweep
  if (currentMillis - lastRadarDraw >= 50) {
    lastRadarDraw = currentMillis;
    sweepAngle += 0.15;
    if (sweepAngle >= 2 * PI) sweepAngle = 0;

    drawRadarScreen();
    digitalWrite(EXTERNAL_LED, LOW); // Turn off blip LED pulse
  }
}