#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

extern "C" {
  #include "user_interface.h"
}

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Hardware Pin Definitions
#define OLED_SDA     D3  // GPIO0
#define OLED_SCL     D4  // GPIO2
#define IR_RECV_PIN  D2  // TSOP OUT (GPIO4)
#define BUTTON_PIN   D5  // Mode switch button (Active LOW)
#define EXTERNAL_LED D1  // Visual indicator LED

// IR Configuration
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;
IRrecv irrecv(IR_RECV_PIN, kCaptureBufferSize, kTimeout, true);
decode_results irResults;

String lastProtocol = "Ready";
String lastHexCode  = "Press Remote";
int lastBits        = 0;

// Wi-Fi Radar Structures & Buffers
#define MAX_TARGETS 8

struct Target {
  uint8_t mac[6];
  int rssi;
  float angle;
  unsigned long lastSeen;
  bool active;
};

Target targets[MAX_TARGETS];
uint8_t currentChannel = 1;
unsigned long lastChannelHop = 0;
unsigned long lastDisplayDraw = 0;
float sweepAngle = 0.0;

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

// Mode Management (0: Wi-Fi Radar, 1: IR Decoder)
enum DeviceMode {
  MODE_RADAR = 0,
  MODE_IR_DECODER = 1
};

DeviceMode currentMode = MODE_RADAR;

// Non-blocking button & LED variables
unsigned long lastButtonCheck = 0;
bool lastButtonReading = HIGH;
bool irBlinkActive = false;
unsigned long irBlinkStart = 0;
const unsigned long IR_BLINK_DURATION = 80;

void registerTarget(uint8_t* mac, int rssi) {
  int targetIndex = -1;
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (targets[i].active && memcmp(targets[i].mac, mac, 6) == 0) {
      targetIndex = i;
      break;
    }
  }

  if (targetIndex == -1) {
    for (int i = 0; i < MAX_TARGETS; i++) {
      if (!targets[i].active) {
        targetIndex = i;
        break;
      }
    }
  }

  if (targetIndex == -1) {
    targetIndex = random(0, MAX_TARGETS);
  }

  memcpy(targets[targetIndex].mac, mac, 6);
  targets[targetIndex].rssi = rssi;
  targets[targetIndex].lastSeen = millis();

  if (!targets[targetIndex].active) {
    targets[targetIndex].angle = random(0, 360) * (PI / 180.0);
  }
  targets[targetIndex].active = true;

  digitalWrite(EXTERNAL_LED, HIGH);
}

void snifferCallback(uint8_t *buf, uint16_t len) {
  if (currentMode != MODE_RADAR || len == 12) return;

  struct SnifferPacket *sniffer = (struct SnifferPacket*) buf;
  int rssi = sniffer->rx_ctrl.rssi;

  if (sniffer->buf[0] == 0x40) { // Probe Request frame
    uint8_t *srcMac = &sniffer->buf[10];
    registerTarget(srcMac, rssi);
  }
}

void configureMode(DeviceMode newMode) {
  currentMode = newMode;
  display.clearDisplay();

  if (currentMode == MODE_RADAR) {
    irrecv.disableIRIn();
    
    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    wifi_set_opmode(STATION_MODE);
    wifi_promiscuous_enable(0);
    wifi_set_promiscuous_rx_cb(snifferCallback);
    wifi_promiscuous_enable(1);
    
    Serial.println("\n[MODE] Switched to 2.4 GHz Wi-Fi Radar");
  } else {
    wifi_promiscuous_enable(0);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    irrecv.setUnknownThreshold(12);
    irrecv.enableIRIn();
    
    Serial.println("\n[MODE] Switched to TSOP IR Decoder");
  }
}

void drawRadarUI() {
  display.clearDisplay();

  const int centerX = 40;
  const int centerY = 32;
  const int maxRadius = 30;

  // Radar scope rings & crosshairs
  display.drawCircle(centerX, centerY, 10, SSD1306_WHITE);
  display.drawCircle(centerX, centerY, 20, SSD1306_WHITE);
  display.drawCircle(centerX, centerY, maxRadius, SSD1306_WHITE);
  display.drawLine(centerX - maxRadius, centerY, centerX + maxRadius, centerY, SSD1306_WHITE);
  display.drawLine(centerX, centerY - maxRadius, centerX, centerY + maxRadius, SSD1306_WHITE);

  // Rotating sweep beam
  int sweepX = centerX + cos(sweepAngle) * maxRadius;
  int sweepY = centerY + sin(sweepAngle) * maxRadius;
  display.drawLine(centerX, centerY, sweepX, sweepY, SSD1306_WHITE);

  int activeCount = 0;
  int closestRSSI = -100;

  for (int i = 0; i < MAX_TARGETS; i++) {
    if (targets[i].active) {
      if (millis() - targets[i].lastSeen > 6000) {
        targets[i].active = false;
        continue;
      }
      activeCount++;
      if (targets[i].rssi > closestRSSI) {
        closestRSSI = targets[i].rssi;
      }

      int dist = map(targets[i].rssi, -95, -35, maxRadius - 2, 4);
      dist = constrain(dist, 4, maxRadius - 2);

      int tx = centerX + cos(targets[i].angle) * dist;
      int ty = centerY + sin(targets[i].angle) * dist;
      display.fillRect(tx - 1, ty - 1, 3, 3, SSD1306_WHITE);
    }
  }

  // Sidebar readout
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
    display.print("SWEEP");
  }

  display.display();
}

void drawDecoderUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("--- IR DECODER ---");

  display.setCursor(0, 16);
  display.print("Proto: ");
  display.println(lastProtocol);

  display.setCursor(0, 28);
  display.print("Code : ");
  display.println(lastHexCode);

  display.setCursor(0, 40);
  display.printf("Bits : %d-bit\n", lastBits);

  display.setCursor(0, 54);
  display.print("TSOP Receiver: Active");

  display.display();
}

void setup() {
  pinMode(EXTERNAL_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_RECV_PIN, INPUT_PULLUP);
  digitalWrite(EXTERNAL_LED, LOW);

  Serial.begin(115200);
  delay(200);

  // Initialize OLED on D3 & D4
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  // Boot into default mode (Radar)
  configureMode(MODE_RADAR);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Debounced Mode Button Handler (D5)
  if (currentMillis - lastButtonCheck >= 50) {
    lastButtonCheck = currentMillis;
    bool reading = digitalRead(BUTTON_PIN);
    if (reading == LOW && lastButtonReading == HIGH) {
      DeviceMode nextMode = (currentMode == MODE_RADAR) ? MODE_IR_DECODER : MODE_RADAR;
      configureMode(nextMode);
    }
    lastButtonReading = reading;
  }

  // 2. Mode-Specific Execution
  if (currentMode == MODE_RADAR) {
    // Channel hopping every 180 ms
    if (currentMillis - lastChannelHop >= 180) {
      lastChannelHop = currentMillis;
      currentChannel++;
      if (currentChannel > 13) currentChannel = 1;
      wifi_set_channel(currentChannel);
    }

    // Refresh Radar Scope (~20 FPS)
    if (currentMillis - lastDisplayDraw >= 50) {
      lastDisplayDraw = currentMillis;
      sweepAngle += 0.15;
      if (sweepAngle >= 2 * PI) sweepAngle = 0;
      drawRadarUI();
      digitalWrite(EXTERNAL_LED, LOW);
    }

  } else {
    // IR Decoder Mode
    if (irrecv.decode(&irResults)) {
      bool isValidSignal = (irResults.decode_type != decode_type_t::UNKNOWN) &&
                           (irResults.bits >= 8) &&
                           (irResults.value != 0);

      if (isValidSignal) {
        lastProtocol = typeToString(irResults.decode_type);
        lastHexCode  = "0x" + uint64ToString(irResults.value, HEX);
        lastBits     = irResults.bits;

        Serial.printf("\n[IR] %s | Code: %s | %d bits\n",
                      lastProtocol.c_str(), lastHexCode.c_str(), lastBits);

        irBlinkActive = true;
        irBlinkStart  = currentMillis;
        digitalWrite(EXTERNAL_LED, HIGH);
      }
      irrecv.resume();
    }

    // Manage non-blocking IR indicator pulse
    if (irBlinkActive && (currentMillis - irBlinkStart >= IR_BLINK_DURATION)) {
      irBlinkActive = false;
      digitalWrite(EXTERNAL_LED, LOW);
    }

    // Refresh IR UI (~10 FPS)
    if (currentMillis - lastDisplayDraw >= 100) {
      lastDisplayDraw = currentMillis;
      drawDecoderUI();
    }
  }
}