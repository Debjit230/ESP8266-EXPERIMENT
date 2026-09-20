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
#include <time.h>

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
#define EXTERNAL_LED D1  // Visual indicator LED (Active HIGH)
#define ONBOARD_LED  D0  // NodeMCU on-board LED (GPIO16, Active LOW)

// AP Config Portal Hotspot Credentials
const char* AP_CONFIG_SSID = "ESP-Sentinel-Config";
const char* AP_CONFIG_PASS = "12345678";

// EEPROM Storage Settings
#define EEPROM_SIZE 128
#define EEPROM_MAGIC 0x5A
#define EEPROM_BRIGHT_ADDR   97
#define EEPROM_AUTODIM_ADDR  98

char target_ssid[33]     = "";
char target_password[65] = "";
uint8_t userBrightness   = 255;  // Default manual level (1 - 255)
bool autoDimEnabled      = false; // Default: Fully manual control

ESP8266WebServer server(80);

// NTP Time Configuration (IST: UTC +5:30 = 19800 seconds)
const long  gmtOffset_sec     = 19800;
const int   daylightOffset_sec = 0;
const char* ntpServer         = "pool.ntp.org";

// Timers & Lock Settings
const unsigned long CLOCK_TIMEOUT_MS = 20000;   // 20 sec -> Clock screensaver
const unsigned long AUTO_LOCK_MS     = 60000;   // 60 sec -> EMO face lock screen
unsigned long lastUserActivity       = 0;
bool isClockModeActive               = false;
bool isDeviceLocked                  = false;

// OLED Hardware Brightness Control
void setOledBrightness(uint8_t contrast) {
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(contrast);
}

// Global Non-Blocking Alert LED Controller
bool alertLedActive = false;
unsigned long alertLedStart = 0;
const unsigned long LED_ALERT_DURATION = 120;

void triggerLedAlert() {
  if (isDeviceLocked) return; // Keep all LEDs completely dark when in EMO Lock Screen

  alertLedActive = true;
  alertLedStart = millis();
  digitalWrite(EXTERNAL_LED, HIGH);
  digitalWrite(ONBOARD_LED, LOW); // Active-LOW: LOW turns it ON
}

// Emotion Engine Definitions
enum EmoState {
  EMO_NORMAL,
  EMO_SUSPICIOUS,
  EMO_SHOCKED,
  EMO_WINK,
  EMO_SLEEP,
  EMO_GOOD_MORNING
};

EmoState currentEmotion = EMO_NORMAL;
unsigned long emotionHoldStartTime = 0;
const unsigned long EMOTION_HOLD_MS = 2200;

void setEmotion(EmoState newEmo) {
  currentEmotion = newEmo;
  emotionHoldStartTime = millis();
}

bool isMorningWindow() {
  time_t tNow = time(nullptr);
  struct tm* timeinfo = localtime(&tNow);
  if (timeinfo->tm_hour == 6) return true;
  if (timeinfo->tm_hour == 7 && timeinfo->tm_min <= 30) return true;
  return false;
}

bool isNightWindow() {
  time_t tNow = time(nullptr);
  struct tm* timeinfo = localtime(&tNow);
  if (timeinfo->tm_hour >= 23 || timeinfo->tm_hour < 6) return true;
  return false;
}

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
DeviceMode resumeMode  = MODE_RADAR;

// Multi-Click & Long-Press Button Engine
unsigned long lastButtonCheck = 0;
bool buttonIsPressed = false;
unsigned long buttonPressStartTime = 0;
bool holdThresholdMet = false;
int clickCount = 0;
unsigned long lastClickTime = 0;
const unsigned long DOUBLE_CLICK_GAP = 350;

// EMO Animation Parameters
unsigned long lastEyeAnim = 0;
int eyeOffsetX = 0;
int eyeOffsetY = 0;
int eyeHeight  = 30;
bool isBlinking = false;
unsigned long blinkStartTime = 0;
int zzzStep = 0;
unsigned long lastZzzAnim = 0;

void loadCredentials() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(0) == EEPROM_MAGIC) {
    for (int i = 0; i < 32; i++) target_ssid[i] = EEPROM.read(1 + i);
    target_ssid[32] = '\0';
    for (int i = 0; i < 64; i++) target_password[i] = EEPROM.read(33 + i);
    target_password[64] = '\0';

    uint8_t storedBright = EEPROM.read(EEPROM_BRIGHT_ADDR);
    if (storedBright >= 1 && storedBright <= 255) {
      userBrightness = storedBright;
    } else {
      userBrightness = 255;
    }

    uint8_t storedDim = EEPROM.read(EEPROM_AUTODIM_ADDR);
    autoDimEnabled = (storedDim == 1);
  } else {
    strncpy(target_ssid, "Airfiber-3rdFloorBachelor", sizeof(target_ssid));
    strncpy(target_password, "Airfiber-3rdfloor", sizeof(target_password));
    userBrightness = 255;
    autoDimEnabled = false;
  }
}

void saveSettings(const String& newSSID, const String& newPass, uint8_t newBright, bool newDim) {
  EEPROM.write(0, EEPROM_MAGIC);
  for (int i = 0; i < 32; i++) {
    EEPROM.write(1 + i, i < (int)newSSID.length() ? newSSID[i] : 0);
  }
  for (int i = 0; i < 64; i++) {
    EEPROM.write(33 + i, i < (int)newPass.length() ? newPass[i] : 0);
  }
  EEPROM.write(EEPROM_BRIGHT_ADDR, newBright);
  EEPROM.write(EEPROM_AUTODIM_ADDR, newDim ? 1 : 0);
  EEPROM.commit();

  newSSID.toCharArray(target_ssid, sizeof(target_ssid));
  newPass.toCharArray(target_password, sizeof(target_password));
  userBrightness = newBright;
  autoDimEnabled = newDim;
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

  if (targetIndex == -1) targetIndex = random(0, MAX_TARGETS);

  memcpy(targets[targetIndex].mac, mac, 6);
  targets[targetIndex].rssi = rssi;
  targets[targetIndex].lastSeen = millis();

  if (!targets[targetIndex].active) {
    targets[targetIndex].angle = random(0, 360) * (PI / 180.0);
  }
  targets[targetIndex].active = true;

  triggerLedAlert();

  if (isDeviceLocked) {
    setEmotion(EMO_SUSPICIOUS);
  }
}

void snifferCallback(uint8_t *buf, uint16_t len) {
  if (len == 12) return;

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
  if (currentMode == MODE_AP_CONFIG) {
    WiFi.mode(WIFI_AP_STA);
    int n = WiFi.scanNetworks();

    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial,sans-serif;background:#121212;color:#eee;text-align:center;padding:20px;}";
    html += "form{background:#1f1f1f;padding:22px;border-radius:12px;display:inline-block;max-width:340px;width:100%;box-sizing:border-box;}";
    html += "select,input[type=password],input[type=text]{width:100%;padding:10px;margin:8px 0 16px 0;border-radius:6px;border:1px solid #333;background:#2a2a2a;color:#fff;box-sizing:border-box;}";
    html += ".slider-container{margin:15px 0 10px 0;text-align:left;}";
    html += ".toggle-container{margin:15px 0 20px 0;text-align:left;display:flex;align-items:center;}";
    html += "input[type=range]{width:100%;margin:10px 0;accent-color:#00bcd4;cursor:pointer;}";
    html += "input[type=checkbox]{width:18px;height:18px;margin-right:10px;accent-color:#00bcd4;cursor:pointer;}";
    html += "input[type=submit]{width:100%;background:#00bcd4;color:#fff;padding:12px;border:none;border-radius:6px;font-weight:bold;font-size:16px;cursor:pointer;}";
    html += "label{font-size:14px;color:#aaa;display:block;text-align:left;font-weight:bold;}";
    html += ".val-badge{float:right;color:#00bcd4;font-size:14px;}";
    html += "</style></head><body><h2>Sentinel AP Setup</h2>";
    html += "<form method='POST' action='/save'>";
    html += "<label>Select Nearby Wi-Fi:</label><select name='ssid'>";

    if (n == 0) {
      html += "<option value=''>No networks found</option>";
    } else {
      for (int i = 0; i < n; ++i) {
        String s = WiFi.SSID(i);
        s.trim();
        if (s.length() > 0) {
          String selected = (s == String(target_ssid)) ? " selected" : "";
          html += "<option value='" + s + "'" + selected + ">" + s + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
        }
      }
    }

    html += "</select><label>Password:</label>";
    html += "<input type='password' name='pass' value='" + String(target_password) + "'><br>";

    // Manual Brightness Slider
    html += "<div class='slider-container'>";
    html += "<label>Manual Brightness: <span id='bVal' class='val-badge'>" + String(userBrightness) + "</span></label>";
    html += "<input type='range' name='bright' min='1' max='255' value='" + String(userBrightness) + "' oninput=\"document.getElementById('bVal').innerText=this.value;\">";
    html += "</div>";

    // Optional Auto-Dim Feature Toggle
    html += "<div class='toggle-container'>";
    html += "<input type='checkbox' id='autodim' name='autodim' value='1'" + String(autoDimEnabled ? " checked" : "") + ">";
    html += "<label for='autodim' style='font-size:13px;cursor:pointer;color:#eee;'>Auto-dim on Lock / Clock</label>";
    html += "</div>";

    html += "<input type='submit' value='Save & Reboot'></form></body></html>";
    server.send(200, "text/html", html);
  } else {
    server.send(200, "text/plain", "ESP-Sentinel Ready");
  }
}

void handleConfigSave() {
  String reqSSID    = server.arg("ssid");
  String reqPass    = server.arg("pass");
  String reqBright  = server.arg("bright");
  bool reqAutoDim   = server.hasArg("autodim");

  uint8_t newBright = userBrightness;
  if (reqBright.length() > 0) {
    int b = reqBright.toInt();
    if (b >= 1 && b <= 255) newBright = (uint8_t)b;
  }

  if (reqSSID.length() > 0) {
    saveSettings(reqSSID, reqPass, newBright, reqAutoDim);
    setOledBrightness(newBright);

    String html = "<html><body style='background:#121212;color:#eee;text-align:center;padding:40px;font-family:Arial;'>";
    html += "<h2>Settings Saved!</h2><p>Brightness: " + String(newBright) + "</p>";
    html += "<p>Auto-dim: " + String(reqAutoDim ? "Enabled" : "Disabled") + "</p><p>Rebooting...</p></body></html>";
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

  // Mode changes keep the exact user manual brightness
  setOledBrightness(userBrightness);

  server.stop();
  wifi_promiscuous_enable(0);
  irrecv.disableIRIn();
  digitalWrite(EXTERNAL_LED, LOW);
  digitalWrite(ONBOARD_LED, HIGH); // Active-LOW: HIGH turns it OFF
  alertLedActive = false;

  switch (currentMode) {
    case MODE_RADAR:
      WiFi.persistent(false);
      WiFi.disconnect();
      WiFi.mode(WIFI_STA);
      wifi_set_opmode(STATION_MODE);
      wifi_promiscuous_enable(0);
      wifi_set_promiscuous_rx_cb(snifferCallback);
      wifi_promiscuous_enable(1);
      break;

    case MODE_SCANNER:
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      totalNetworksFound = 0;
      triggerWifiScan();
      lastScanTime = millis();
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
      break;

    case MODE_IR_DECODER:
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      irrecv.setUnknownThreshold(12);
      irrecv.enableIRIn();
      break;

    case MODE_AP_CONFIG: {
      WiFi.persistent(false);
      WiFi.disconnect(true);
      delay(50);
      WiFi.mode(WIFI_AP_STA);

      IPAddress local_ip(192, 168, 4, 1);
      IPAddress gateway(192, 168, 4, 1);
      IPAddress subnet(255, 255, 255, 0);
      WiFi.softAPConfig(local_ip, gateway, subnet);
      WiFi.softAP(AP_CONFIG_SSID, AP_CONFIG_PASS, 1, 0, 4);

      server.begin();
      break;
    }
  }
}

void enterLockScreen() {
  isDeviceLocked = true;
  isClockModeActive = false;
  resumeMode = currentMode;

  // Turn off both external and on-board LEDs completely
  digitalWrite(EXTERNAL_LED, LOW);
  digitalWrite(ONBOARD_LED, HIGH); // Active-LOW: HIGH turns it OFF
  alertLedActive = false;

  // Only dim if auto-dim was explicitly enabled in web portal
  if (autoDimEnabled) {
    uint8_t dimLevel = map(userBrightness, 1, 255, 1, 35);
    setOledBrightness(dimLevel);
  } else {
    setOledBrightness(userBrightness);
  }

  if (isNightWindow()) {
    currentEmotion = EMO_SLEEP;
  } else if (isMorningWindow()) {
    setEmotion(EMO_GOOD_MORNING);
  } else {
    currentEmotion = EMO_NORMAL;
  }

  display.clearDisplay();
  display.display();
}

void unlockDevice() {
  isDeviceLocked = false;
  lastUserActivity = millis();

  // Always restore user manual brightness
  setOledBrightness(userBrightness);

  display.clearDisplay();
  configureMode(resumeMode);
}

// Emotional Engine Renderer
void drawEmoFace() {
  unsigned long now = millis();

  if (currentEmotion != EMO_NORMAL && currentEmotion != EMO_SLEEP) {
    if (now - emotionHoldStartTime > EMOTION_HOLD_MS) {
      currentEmotion = isNightWindow() ? EMO_SLEEP : EMO_NORMAL;
    }
  } else {
    if (isNightWindow()) {
      currentEmotion = EMO_SLEEP;
    }
  }

  display.clearDisplay();

  const int eyeW = 28;
  const int leftEyeBaseX = 26;
  const int rightEyeBaseX = 74;
  const int eyeBaseY = 17;

  // 1. GOOD MORNING STATE
  if (currentEmotion == EMO_GOOD_MORNING) {
    display.fillRoundRect(leftEyeBaseX, eyeBaseY + 2, eyeW, 26, 8, SSD1306_WHITE);
    display.fillRoundRect(rightEyeBaseX, eyeBaseY + 2, eyeW, 26, 8, SSD1306_WHITE);
    display.fillRect(leftEyeBaseX, eyeBaseY + 16, eyeW, 14, SSD1306_BLACK);
    display.fillRect(rightEyeBaseX, eyeBaseY + 16, eyeW, 14, SSD1306_BLACK);

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(22, 50);
    display.print("GOOD MORNING!");

    display.display();
    return;
  }

  // 2. SLEEPING STATE
  if (currentEmotion == EMO_SLEEP) {
    display.fillRect(leftEyeBaseX, eyeBaseY + 14, eyeW, 3, SSD1306_WHITE);
    display.fillRect(rightEyeBaseX, eyeBaseY + 14, eyeW, 3, SSD1306_WHITE);

    if (now - lastZzzAnim > 350) {
      lastZzzAnim = now;
      zzzStep = (zzzStep + 1) % 4;
    }
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    if (zzzStep >= 1) { display.setCursor(98, 24); display.print("z"); }
    if (zzzStep >= 2) { display.setCursor(106, 15); display.print("Z"); }
    if (zzzStep >= 3) { display.setCursor(114, 6);  display.print("Z"); }

    display.display();
    return;
  }

  // 3. SHOCKED STATE
  if (currentEmotion == EMO_SHOCKED) {
    display.fillCircle(leftEyeBaseX + 14, eyeBaseY + 15, 17, SSD1306_WHITE);
    display.fillCircle(rightEyeBaseX + 14, eyeBaseY + 15, 17, SSD1306_WHITE);
    display.fillCircle(leftEyeBaseX + 14, eyeBaseY + 15, 5, SSD1306_BLACK);
    display.fillCircle(rightEyeBaseX + 14, eyeBaseY + 15, 5, SSD1306_BLACK);

    display.display();
    return;
  }

  // 4. SUSPICIOUS / SQUINT STATE
  if (currentEmotion == EMO_SUSPICIOUS) {
    display.fillRoundRect(leftEyeBaseX, eyeBaseY + 8, eyeW, 16, 4, SSD1306_WHITE);
    display.fillRoundRect(rightEyeBaseX, eyeBaseY + 8, eyeW, 16, 4, SSD1306_WHITE);

    display.fillTriangle(leftEyeBaseX, eyeBaseY + 8, leftEyeBaseX + eyeW, eyeBaseY + 8, leftEyeBaseX + eyeW, eyeBaseY + 15, SSD1306_BLACK);
    display.fillTriangle(rightEyeBaseX, eyeBaseY + 8, rightEyeBaseX + eyeW, eyeBaseY + 8, rightEyeBaseX, eyeBaseY + 15, SSD1306_BLACK);

    display.display();
    return;
  }

  // 5. WINK STATE
  if (currentEmotion == EMO_WINK) {
    display.fillRoundRect(leftEyeBaseX, eyeBaseY, eyeW, 30, 7, SSD1306_WHITE);
    display.fillRoundRect(rightEyeBaseX, eyeBaseY + 14, eyeW, 4, 2, SSD1306_WHITE);

    display.display();
    return;
  }

  // 6. NORMAL IDLE STATE
  if (!isBlinking && (now - lastEyeAnim > random(2500, 5000))) {
    isBlinking = true;
    blinkStartTime = now;
    lastEyeAnim = now;

    eyeOffsetX = random(-7, 8);
    eyeOffsetY = random(-4, 5);
  }

  if (isBlinking) {
    if (now - blinkStartTime < 100) {
      eyeHeight = 4;
    } else {
      isBlinking = false;
      eyeHeight = 30;
    }
  }

  int lx = constrain(leftEyeBaseX + eyeOffsetX, 4, 46);
  int rx = constrain(rightEyeBaseX + eyeOffsetX, 54, 96);
  int y  = constrain(eyeBaseY + eyeOffsetY, 6, 28);

  display.fillRoundRect(lx, y + (30 - eyeHeight) / 2, eyeW, eyeHeight, 7, SSD1306_WHITE);
  display.fillRoundRect(rx, y + (30 - eyeHeight) / 2, eyeW, eyeHeight, 7, SSD1306_WHITE);

  display.display();
}

void drawClockUI() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  char timeStr[10];
  int hour12 = timeinfo->tm_hour % 12;
  if (hour12 == 0) hour12 = 12;
  const char* ampm = (timeinfo->tm_hour >= 12) ? "PM" : "AM";

  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hour12, timeinfo->tm_min, timeinfo->tm_sec);

  display.setTextSize(2);
  display.setCursor(4, 14);
  display.print(timeStr);

  display.setTextSize(1);
  display.setCursor(104, 20);
  display.print(ampm);

  const char* days[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  const char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

  display.setTextSize(1);
  display.setCursor(6, 40);
  display.printf("%s, %02d %s %04d", 
                 days[timeinfo->tm_wday], 
                 timeinfo->tm_mday, 
                 months[timeinfo->tm_mon], 
                 timeinfo->tm_year + 1900);

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
  display.setCursor(0, 24);
  display.println("Pass: 12345678");

  display.setCursor(0, 36);
  display.printf("Bright: %d/255\n", userBrightness);
  display.setCursor(0, 46);
  display.printf("Auto-Dim: %s\n", autoDimEnabled ? "ON" : "OFF");

  display.setCursor(0, 56);
  display.println("http://192.168.4.1");

  display.display();
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

void syncTimeAtStartup() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("CONNECTING AP...");
  display.setCursor(0, 26);
  display.println(target_ssid[0] ? target_ssid : "[NO AP STORED]");
  display.setCursor(0, 44);
  display.println("Hold D5: Setup AP");
  display.display();

  if (!target_ssid[0]) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(target_ssid, target_password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(100);
    if (digitalRead(BUTTON_PIN) == LOW) {
      configureMode(MODE_AP_CONFIG);
      return;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    display.clearDisplay();
    display.setCursor(0, 18);
    display.println("Wi-Fi Connected!");
    display.setCursor(0, 34);
    display.println("Syncing NTP Time...");
    display.display();

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    time_t now = time(nullptr);
    start = millis();
    while (now < 100000 && millis() - start < 4000) {
      delay(200);
      now = time(nullptr);
    }
  }
}

void setup() {
  pinMode(EXTERNAL_LED, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_RECV_PIN, INPUT_PULLUP);

  // Set all LEDs to dark off-state by default
  digitalWrite(EXTERNAL_LED, LOW);
  digitalWrite(ONBOARD_LED, HIGH); // Active-LOW: HIGH turns it OFF

  Serial.begin(115200);
  delay(100);

  loadCredentials();

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleConfigSave);

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(2);
  display.clearDisplay();
  display.display();

  // Apply user manual brightness from EEPROM
  setOledBrightness(userBrightness);

  syncTimeAtStartup();

  lastUserActivity = millis();

  if (isMorningWindow()) {
    enterLockScreen();
    setEmotion(EMO_GOOD_MORNING);
  } else if (currentMode != MODE_AP_CONFIG) {
    configureMode(MODE_RADAR);
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Universal Non-Blocking Alert LED Pulse Shutoff
  if (alertLedActive && (currentMillis - alertLedStart >= LED_ALERT_DURATION)) {
    alertLedActive = false;
    digitalWrite(EXTERNAL_LED, LOW);
    digitalWrite(ONBOARD_LED, HIGH); // Active-LOW: HIGH turns it OFF
  }

  // 2. Button Engine (Single-Click, Fast Double-Click, and 2-Second Hold)
  if (currentMillis - lastButtonCheck >= 25) {
    lastButtonCheck = currentMillis;
    bool pinState = (digitalRead(BUTTON_PIN) == LOW);

    if (pinState && !buttonIsPressed) {
      buttonIsPressed = true;
      buttonPressStartTime = currentMillis;
      holdThresholdMet = false;
    } 
    else if (pinState && buttonIsPressed) {
      unsigned long heldTime = currentMillis - buttonPressStartTime;

      if (!isDeviceLocked && !holdThresholdMet) {
        if (heldTime >= 2000) {
          holdThresholdMet = true;
          clickCount = 0;
          lastUserActivity = currentMillis;
          isClockModeActive = false;
          configureMode(MODE_AP_CONFIG);
        } else if (heldTime >= 400) {
          display.fillRect(10, 56, (heldTime - 400) * 108 / 1600, 4, SSD1306_WHITE);
          display.display();
        }
      }
    } 
    else if (!pinState && buttonIsPressed) {
      buttonIsPressed = false;
      unsigned long duration = currentMillis - buttonPressStartTime;

      if (duration < 600 && !holdThresholdMet) {
        clickCount++;
        lastClickTime = currentMillis;

        if (clickCount == 2) {
          clickCount = 0;
          if (isDeviceLocked) {
            unlockDevice();
          } else {
            enterLockScreen();
          }
        }
      }
    }

    if (clickCount == 1 && (currentMillis - lastClickTime > DOUBLE_CLICK_GAP)) {
      clickCount = 0;
      if (!isDeviceLocked) {
        lastUserActivity = currentMillis;

        if (isClockModeActive) {
          isClockModeActive = false;
          setOledBrightness(userBrightness);
          display.clearDisplay();
        } else {
          DeviceMode nextMode;
          if (currentMode == MODE_RADAR)             nextMode = MODE_SCANNER;
          else if (currentMode == MODE_SCANNER)     nextMode = MODE_RF_TRIPWIRE;
          else if (currentMode == MODE_RF_TRIPWIRE) nextMode = MODE_IR_DECODER;
          else                                      nextMode = MODE_RADAR;

          configureMode(nextMode);
        }
      }
    }
  }

  // 3. Automated Locks and Screen Savers
  if (!isDeviceLocked && currentMode != MODE_AP_CONFIG) {
    if (currentMillis - lastUserActivity >= AUTO_LOCK_MS) {
      enterLockScreen();
    } else if (!isClockModeActive && (currentMillis - lastUserActivity >= CLOCK_TIMEOUT_MS)) {
      isClockModeActive = true;
      if (autoDimEnabled) {
        uint8_t dimLevel = map(userBrightness, 1, 255, 1, 35);
        setOledBrightness(dimLevel);
      } else {
        setOledBrightness(userBrightness);
      }
      display.clearDisplay();
    }
  }

  // 4. UI Display Handlers
  if (isDeviceLocked) {
    if (currentMillis - lastDisplayDraw >= 40) {
      lastDisplayDraw = currentMillis;
      drawEmoFace();
    }
  } else if (isClockModeActive) {
    if (currentMillis - lastDisplayDraw >= 500) {
      lastDisplayDraw = currentMillis;
      drawClockUI();
    }
  }

  // 5. Active & Background Logic Execution
  switch (currentMode) {
    case MODE_RADAR: {
      if (currentMillis - lastChannelHop >= 180) {
        lastChannelHop = currentMillis;
        currentChannel++;
        if (currentChannel > 13) currentChannel = 1;
        wifi_set_channel(currentChannel);
      }

      if (!isDeviceLocked && !isClockModeActive && (currentMillis - lastDisplayDraw >= 50)) {
        lastDisplayDraw = currentMillis;
        sweepAngle += 0.15;
        if (sweepAngle >= 2 * PI) sweepAngle = 0;
        drawRadarUI();
      }
      break;
    }

    case MODE_SCANNER: {
      if (!isDeviceLocked) {
        if (!scanningInProgress && (currentMillis - lastScanTime >= SCAN_INTERVAL)) {
          lastScanTime = currentMillis;
          triggerWifiScan();
        }

        if (!isClockModeActive && (currentMillis - lastDisplayDraw >= 200)) {
          lastDisplayDraw = currentMillis;
          drawScannerUI();
        }
      }
      break;
    }

    case MODE_RF_TRIPWIRE: {
      if (WiFi.status() != WL_CONNECTED) {
        if (!isDeviceLocked && !isClockModeActive && (currentMillis - lastDisplayDraw >= 300)) {
          lastDisplayDraw = currentMillis;
          display.clearDisplay();
          display.setTextColor(SSD1306_WHITE);
          display.setTextSize(1);
          display.setCursor(0, 24);
          display.println("Connecting to AP...");
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

        if (isCalibrating) {
          if (delta > maxNoiseObserved) {
            maxNoiseObserved = delta;
          }

          unsigned long elapsed = currentMillis - calibrationStartTime;
          if (elapsed >= 5000) {
            dynamicThreshold = maxNoiseObserved + 1.0;
            if (dynamicThreshold < 2.0) dynamicThreshold = 2.0;
            isCalibrating = false;
          }

          if (!isDeviceLocked && !isClockModeActive) {
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
          }
          break;
        }

        if (delta >= dynamicThreshold) {
          lastMotionDetected = currentMillis;
          triggerLedAlert();

          if (isDeviceLocked) {
            setEmotion(EMO_SHOCKED);
          }
        }

        bool isMotionActive = (currentMillis - lastMotionDetected < ALARM_HOLD_TIME);

        int yVal = 42 - (int)(((float)currentRssi - baselineRSSI) * 3.5);
        yVal = constrain(yVal, 22, 62);

        waveBuffer[waveIndex] = yVal;
        waveIndex = (waveIndex + 1) % WAVE_POINTS;

        if (!isDeviceLocked && !isClockModeActive) {
          drawTripwireUI(delta, currentRssi, isMotionActive);
        }
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

          triggerLedAlert();

          if (isDeviceLocked) {
            setEmotion(EMO_WINK);
          } else {
            lastUserActivity = currentMillis;
            if (isClockModeActive) {
              isClockModeActive = false;
              setOledBrightness(userBrightness);
              display.clearDisplay();
            }
          }
        }
        irrecv.resume();
      }

      if (!isDeviceLocked && !isClockModeActive && (currentMillis - lastDisplayDraw >= 100)) {
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