#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
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

// AP Configuration Portal Credentials
const char* AP_CONFIG_SSID = "ESP-Sentinel-Config";
const char* AP_CONFIG_PASS = "12345678";

// EEPROM Storage Settings
#define EEPROM_SIZE 96
#define EEPROM_MAGIC 0x5A
char target_ssid[33]     = "";
char target_password[65] = "";

// Web Server Instance on Port 80
ESP8266WebServer server(80);

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

// Wi-Fi AP Scanner Storage
int totalNetworksFound = 0;
unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 5000;
bool scanningInProgress = false;

// Device-Free RF Tripwire Adaptive Motion Sensing
const unsigned long SAMPLE_RATE_MS = 50;
const float EMA_ALPHA = 0.08;

float baselineRSSI = 0.0;
bool baselineInitialized = false;
unsigned long lastSampleTime = 0;
unsigned long lastMotionDetected = 0;
const unsigned long ALARM_HOLD_TIME = 1500;

// Auto-Calibration Parameters
bool isCalibrating = false;
unsigned long calibrationStartTime = 0;
float maxNoiseObserved = 0.0;
float dynamicThreshold = 2.4;

#define WAVE_POINTS 128
int waveBuffer[WAVE_POINTS];
int waveIndex = 0;

// Operational Modes
enum DeviceMode {
  MODE_RADAR = 0,
  MODE_SCANNER = 1,
  MODE_RF_TRIPWIRE = 2,
  MODE_IR_DECODER = 3,
  MODE_AP_CONFIG = 4
};

DeviceMode currentMode = MODE_RADAR;

// Non-blocking Button Timing & Long Press Detection
unsigned long lastButtonCheck = 0;
bool lastButtonReading = HIGH;
unsigned long buttonPressStartTime = 0;
bool longPressTriggered = false;

bool irBlinkActive = false;
unsigned long irBlinkStart = 0;
const unsigned long IR_BLINK_DURATION = 80;

void loadCredentials() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(0) == EEPROM_MAGIC) {
    for (int i = 0; i < 32; i++) target_ssid[i] = EEPROM.read(1 + i);
    target_ssid[32] = '\0';
    for (int i = 0; i < 64; i++) target_password[i] = EEPROM.read(33 + i);
    target_password[64] = '\0';
  } else {
    strncpy(target_ssid, "Airfiber-3rdFloorBachelor", sizeof(target_ssid));
    strncpy(target_password, "Airfiber-3rdfloor", sizeof(target_password));
  }
}

void saveCredentials(const String& newSSID, const String& newPass) {
  EEPROM.write(0, EEPROM_MAGIC);
  for (int i = 0; i < 32; i++) {
    EEPROM.write(1 + i, i < (int)newSSID.length() ? newSSID[i] : 0);
  }
  for (int i = 0; i < 64; i++) {
    EEPROM.write(33 + i, i < (int)newPass.length() ? newPass[i] : 0);
  }
  EEPROM.commit();
  newSSID.toCharArray(target_ssid, sizeof(target_ssid));
  newPass.toCharArray(target_password, sizeof(target_password));
}

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

  if (sniffer->buf[0] == 0x40) {
    uint8_t *srcMac = &sniffer->buf[10];
    registerTarget(srcMac, rssi);
  }
}

void triggerWifiScan() {
  scanningInProgress = true;
  WiFi.scanNetworksAsync([](int networksFound) {
    totalNetworksFound = networksFound;
    scanningInProgress = false;
  });
}

void handleRoot() {
  int n = WiFi.scanNetworks();
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial,sans-serif;background:#121212;color:#eee;text-align:center;padding:20px;}";
  html += "form{background:#1f1f1f;padding:20px;border-radius:10px;display:inline-block;max-width:320px;width:100%;box-shadow:0 0 10px rgba(0,0,0,0.5);}";
  html += "select,input{width:90%;padding:10px;margin:10px 0;border-radius:5px;border:none;background:#2a2a2a;color:#fff;}";
  html += "input[type=submit]{background:#00bcd4;color:#fff;font-weight:bold;cursor:pointer;}";
  html += "</style></head><body>";
  html += "<h2>AP Configuration</h2>";
  html += "<form method='POST' action='/save'>";
  html += "<label>Select Nearby Wi-Fi:</label><br><select name='ssid'>";

  for (int i = 0; i < n; ++i) {
    String s = WiFi.SSID(i);
    s.trim();
    if (s.length() > 0) {
      html += "<option value='" + s + "'>" + s + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
    }
  }

  html += "</select><br>";
  html += "<label>Wi-Fi Password:</label><br><input type='password' name='pass' placeholder='Enter Password'><br>";
  html += "<input type='submit' value='Save & Connect'>";
  html += "</form></body></html>";

  server.send(200, "text/html", html);
}

void handleSave() {
  String reqSSID = server.arg("ssid");
  String reqPass = server.arg("pass");

  if (reqSSID.length() > 0) {
    saveCredentials(reqSSID, reqPass);
    String html = "<html><body style='background:#121212;color:#eee;text-align:center;padding:40px;font-family:Arial;'>";
    html += "<h2>Credentials Saved!</h2><p>Rebooting into Tripwire Mode...</p></body></html>";
    server.send(200, "text/html", html);
    delay(1500);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "SSID cannot be empty.");
  }
}

void configureMode(DeviceMode newMode) {
  currentMode = newMode;
  display.clearDisplay();

  // Reset Subsystems
  server.stop();
  wifi_promiscuous_enable(0);
  irrecv.disableIRIn();
  digitalWrite(EXTERNAL_LED, LOW);

  switch (currentMode) {
    case MODE_RADAR:
      WiFi.persistent(false);
      WiFi.disconnect();
      WiFi.mode(WIFI_STA);
      wifi_set_opmode(STATION_MODE);
      wifi_promiscuous_enable(0);
      wifi_set_promiscuous_rx_cb(snifferCallback);
      wifi_promiscuous_enable(1);
      Serial.println(F("\n[MODE 1/5] 2.4 GHz Wi-Fi Radar Scope"));
      break;

    case MODE_SCANNER:
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      totalNetworksFound = 0;
      triggerWifiScan();
      lastScanTime = millis();
      Serial.println(F("\n[MODE 2/5] Wi-Fi AP Scanner"));
      break;

    case MODE_RF_TRIPWIRE:
      WiFi.mode(WIFI_STA);
      WiFi.setSleepMode(WIFI_NONE_SLEEP);
      WiFi.begin(target_ssid, target_password);
      baselineInitialized = false;
      isCalibrating = false;
      calibrationStartTime = 0;
      maxNoiseObserved = 0.0;
      for (int i = 0; i < WAVE_POINTS; i++) waveBuffer[i] = 42;
      Serial.println(F("\n[MODE 3/5] Device-Free RF Motion Tripwire"));
      break;

    case MODE_IR_DECODER:
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      irrecv.setUnknownThreshold(12);
      irrecv.enableIRIn();
      Serial.println(F("\n[MODE 4/5] TSOP IR Remote Decoder"));
      break;

    case MODE_AP_CONFIG:
      WiFi.disconnect();
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(AP_CONFIG_SSID, AP_CONFIG_PASS);
      server.on("/", handleRoot);
      server.on("/save", HTTP_POST, handleSave);
      server.begin();
      Serial.println(F("\n[MODE 5/5] Secured AP Web Configuration Portal Started"));
      break;
  }
}

void drawRadarUI() {
  display.clearDisplay();

  const int centerX = 40;
  const int centerY = 32;
  const int maxRadius = 30;

  display.drawCircle(centerX, centerY, 10, SSD1306_WHITE);
  display.drawCircle(centerX, centerY, 20, SSD1306_WHITE);
  display.drawCircle(centerX, centerY, maxRadius, SSD1306_WHITE);
  display.drawLine(centerX - maxRadius, centerY, centerX + maxRadius, centerY, SSD1306_WHITE);
  display.drawLine(centerX, centerY - maxRadius, centerX, centerY + maxRadius, SSD1306_WHITE);

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

void drawScannerUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.printf("NETWORKS FOUND: %d", totalNetworksFound);
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  int yPos = 12;
  int displayedCount = 0;

  for (int i = 0; i < totalNetworksFound && displayedCount < 4; i++) {
    String ssidName = WiFi.SSID(i);
    ssidName.trim();
    if (ssidName.length() == 0) continue;
    if (ssidName.length() > 11) ssidName = ssidName.substring(0, 11);

    display.setCursor(0, yPos);
    display.printf("%-11s %2ddB", ssidName.c_str(), WiFi.RSSI(i));
    yPos += 11;
    displayedCount++;
  }

  if (displayedCount == 0) {
    display.setCursor(0, 24);
    display.println(scanningInProgress ? "Scanning..." : "No visible APs");
  }

  display.setCursor(0, 56);
  display.print(scanningInProgress ? "Status: Scanning..." : "Status: Active");

  display.display();
}

void drawTripwireUI(float delta, int currentRssi, bool motionAlert) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.printf("TRIPWIRE: %2ddBm", currentRssi);
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  if (motionAlert) {
    display.fillRect(0, 11, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(18, 12);
    display.print("** MOTION ALERT **");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(0, 12);
    display.printf("D:%.1f TH:%.1f", delta, dynamicThreshold);
  }

  display.drawFastHLine(0, 42, 128, SSD1306_WHITE);

  for (int x = 0; x < WAVE_POINTS - 1; x++) {
    int idx1 = (waveIndex + x) % WAVE_POINTS;
    int idx2 = (waveIndex + x + 1) % WAVE_POINTS;
    display.drawLine(x, waveBuffer[idx1], x + 1, waveBuffer[idx2], SSD1306_WHITE);
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
  display.print("TSOP Receiver: Ready");

  display.display();
}

void drawConfigUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("--- AP CONFIG ---");
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  display.setCursor(0, 13);
  display.println("SSID: ESP-Sentinel");
  display.setCursor(0, 23);
  display.println("Pass: 12345678");

  display.setCursor(0, 38);
  display.println("Open in Browser:");
  display.setCursor(0, 48);
  display.println("http://192.168.4.1");

  display.display();
}

void setup() {
  pinMode(EXTERNAL_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_RECV_PIN, INPUT_PULLUP);
  digitalWrite(EXTERNAL_LED, LOW);

  Serial.begin(115200);
  delay(100);

  loadCredentials();

  // Initialize OLED (180-degree inverted view)
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(2);
  display.clearDisplay();
  display.display();

  // Boot into default mode (Wi-Fi Radar Scope)
  configureMode(MODE_RADAR);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Dual-Action Button Handler (Short-press cycles modes; Hold >= 2s opens AP config)
  if (currentMillis - lastButtonCheck >= 40) {
    lastButtonCheck = currentMillis;
    bool reading = digitalRead(BUTTON_PIN);

    if (reading == LOW && lastButtonReading == HIGH) {
      buttonPressStartTime = currentMillis;
      longPressTriggered = false;
    } else if (reading == LOW && !longPressTriggered) {
      if (currentMillis - buttonPressStartTime >= 2000) {
        longPressTriggered = true;
        configureMode(MODE_AP_CONFIG);
      }
    } else if (reading == HIGH && lastButtonReading == LOW) {
      if (!longPressTriggered) {
        DeviceMode nextMode;
        if (currentMode == MODE_RADAR)             nextMode = MODE_SCANNER;
        else if (currentMode == MODE_SCANNER)     nextMode = MODE_RF_TRIPWIRE;
        else if (currentMode == MODE_RF_TRIPWIRE) nextMode = MODE_IR_DECODER;
        else                                      nextMode = MODE_RADAR;

        configureMode(nextMode);
      }
    }
    lastButtonReading = reading;
  }

  // 2. Active Mode Execution
  switch (currentMode) {
    case MODE_RADAR: {
      if (currentMillis - lastChannelHop >= 180) {
        lastChannelHop = currentMillis;
        currentChannel++;
        if (currentChannel > 13) currentChannel = 1;
        wifi_set_channel(currentChannel);
      }

      if (currentMillis - lastDisplayDraw >= 50) {
        lastDisplayDraw = currentMillis;
        sweepAngle += 0.15;
        if (sweepAngle >= 2 * PI) sweepAngle = 0;
        drawRadarUI();
        digitalWrite(EXTERNAL_LED, LOW);
      }
      break;
    }

    case MODE_SCANNER: {
      if (!scanningInProgress && (currentMillis - lastScanTime >= SCAN_INTERVAL)) {
        lastScanTime = currentMillis;
        triggerWifiScan();
      }

      if (currentMillis - lastDisplayDraw >= 200) {
        lastDisplayDraw = currentMillis;
        drawScannerUI();
      }
      break;
    }

    case MODE_RF_TRIPWIRE: {
      if (WiFi.status() != WL_CONNECTED) {
        if (currentMillis - lastDisplayDraw >= 300) {
          lastDisplayDraw = currentMillis;
          display.clearDisplay();
          display.setTextColor(SSD1306_WHITE);
          display.setTextSize(1);
          display.setCursor(0, 16);
          display.println("Connecting to AP:");
          display.setCursor(0, 30);
          display.println(target_ssid[0] ? target_ssid : "[None Set]");
          display.setCursor(0, 48);
          display.println("Hold D5 for Setup");
          display.display();
        }
        break;
      }

      if (!baselineInitialized) {
        baselineRSSI = (float)WiFi.RSSI();
        baselineInitialized = true;
        isCalibrating = true;
        calibrationStartTime = currentMillis;
        maxNoiseObserved = 0.0;
        break;
      }

      if (currentMillis - lastSampleTime >= SAMPLE_RATE_MS) {
        lastSampleTime = currentMillis;
        int currentRssi = WiFi.RSSI();

        baselineRSSI = (EMA_ALPHA * (float)currentRssi) + ((1.0 - EMA_ALPHA) * baselineRSSI);
        float delta = abs((float)currentRssi - baselineRSSI);

        // Phase 1: 5-Second Noise Floor Calibration Window
        if (isCalibrating) {
          if (delta > maxNoiseObserved) {
            maxNoiseObserved = delta;
          }

          unsigned long elapsed = currentMillis - calibrationStartTime;
          if (elapsed >= 5000) {
            dynamicThreshold = maxNoiseObserved + 1.0;
            if (dynamicThreshold < 2.0) dynamicThreshold = 2.0;
            isCalibrating = false;
            Serial.printf("\n[CALIB DONE] Peak Noise: %.2fdB | Auto TH: %.2fdB\n", maxNoiseObserved, dynamicThreshold);
          }

          display.clearDisplay();
          display.setTextColor(SSD1306_WHITE);
          display.setTextSize(1);
          display.setCursor(0, 8);
          display.println("CALIBRATING ROOM...");
          display.setCursor(0, 24);
          display.printf("Remain: %lus", (5000 - elapsed) / 1000 + 1);
          display.setCursor(0, 38);
          display.printf("Noise : %.1fdB", maxNoiseObserved);
          display.setCursor(0, 50);
          display.println("Keep area clear!");
          display.display();
          break;
        }

        // Phase 2: Active Motion Tripwire Detection
        if (delta >= dynamicThreshold) {
          lastMotionDetected = currentMillis;
          digitalWrite(EXTERNAL_LED, HIGH);
        }

        bool isMotionActive = (currentMillis - lastMotionDetected < ALARM_HOLD_TIME);
        if (!isMotionActive) {
          digitalWrite(EXTERNAL_LED, LOW);
        }

        int yVal = 42 - (int)(((float)currentRssi - baselineRSSI) * 3.5);
        yVal = constrain(yVal, 22, 62);

        waveBuffer[waveIndex] = yVal;
        waveIndex = (waveIndex + 1) % WAVE_POINTS;

        drawTripwireUI(delta, currentRssi, isMotionActive);
      }
      break;
    }

    case MODE_IR_DECODER: {
      if (irrecv.decode(&irResults)) {
        bool isValidSignal = (irResults.decode_type != decode_type_t::UNKNOWN) &&
                             (irResults.bits >= 8) &&
                             (irResults.value != 0);

        if (isValidSignal) {
          lastProtocol = typeToString(irResults.decode_type);
          lastHexCode  = "0x" + uint64ToString(irResults.value, HEX);
          lastBits     = irResults.bits;

          Serial.printf("[IR] %s | Code: %s | %d bits\n",
                        lastProtocol.c_str(), lastHexCode.c_str(), lastBits);

          irBlinkActive = true;
          irBlinkStart  = currentMillis;
          digitalWrite(EXTERNAL_LED, HIGH);
        }
        irrecv.resume();
      }

      if (irBlinkActive && (currentMillis - irBlinkStart >= IR_BLINK_DURATION)) {
        irBlinkActive = false;
        digitalWrite(EXTERNAL_LED, LOW);
      }

      if (currentMillis - lastDisplayDraw >= 100) {
        lastDisplayDraw = currentMillis;
        drawDecoderUI();
      }
      break;
    }

    case MODE_AP_CONFIG: {
      server.handleClient();

      if (currentMillis - lastDisplayDraw >= 300) {
        lastDisplayDraw = currentMillis;
        drawConfigUI();
      }
      break;
    }
  }
}