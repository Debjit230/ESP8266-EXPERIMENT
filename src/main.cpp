#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin configurations
#define OLED_SDA     D3  // GPIO0
#define OLED_SCL     D4  // GPIO2
#define EXTERNAL_LED D1  // External LED indicator via 220-ohm resistor

const char* target_ssid = "Airfiber-3rdFloorBachelor";
const char* target_password = "Airfiber-3rdfloor"; // Enter your hotspot password


unsigned long previousMillis = 0;
unsigned long lastWifiCheck = 0;
bool ledState = false;

void oledPrint(const String& line1, const String& line2 = "", const String& line3 = "") {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.println(line1);
  if (line2.length() > 0) display.println(line2);
  if (line3.length() > 0) display.println(line3);
  display.display();

  Serial.println(line1);
  if (line2.length() > 0) Serial.println(line2);
  if (line3.length() > 0) Serial.println(line3);
}

// 4 Fast Blinks on startup
void bootIndicator() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(EXTERNAL_LED, HIGH);
    delay(60);
    digitalWrite(EXTERNAL_LED, LOW);
    delay(60);
  }
}

void setup() {
  pinMode(EXTERNAL_LED, OUTPUT);
  digitalWrite(EXTERNAL_LED, LOW);

  // 1. Boot Indicator: 4 fast flashes on external LED
  bootIndicator();

  Serial.begin(115200);
  delay(200);

  // 2. Initialize I2C explicitly on D3 (SDA) and D4 (SCL)
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[!] SSD1306 allocation failed. Check D3/D4 connections and 0x3C address!"));
  } else {
    display.clearDisplay();
    display.display();
  }

  oledPrint("ESP8266 System Boot", "I2C: SDA->D3, SCL->D4", "Scanning 2.4GHz APs...");
  delay(1000);

  // 3. Scan nearby networks
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks(false, true);
  String scanMsg = "Found APs: " + String(n);
  oledPrint("Wi-Fi Scan Complete", scanMsg, "Connecting to AP...");
  delay(1000);

  // 4. Connect to Hotspot
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(target_ssid, target_password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 25) {
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    oledPrint("Wi-Fi: CONNECTED", "IP: " + WiFi.localIP().toString(), "RSSI: " + String(WiFi.RSSI()) + " dBm");
  } else {
    oledPrint("Wi-Fi: SEARCHING", "Background reconnecting", "SSID: " + String(target_ssid));
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Wi-Fi Reconnection Watchdog (every 5 seconds)
  if (currentMillis - lastWifiCheck >= 5000) {
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
      oledPrint("Wi-Fi: RECONNECTING", "Searching for AP...", "SSID: " + String(target_ssid));
    }
  }

  // External LED Status Blink (1s slow if connected, 150ms fast if connecting)
  bool isConnected = (WiFi.status() == WL_CONNECTED);
  unsigned long blinkInterval = isConnected ? 1000 : 150;

  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(EXTERNAL_LED, ledState ? HIGH : LOW);

    // Refresh OLED state once connected
    static bool updatedScreen = false;
    if (isConnected && !updatedScreen) {
      oledPrint("Status: ONLINE", "IP: " + WiFi.localIP().toString(), "RSSI: " + String(WiFi.RSSI()) + " dBm");
      updatedScreen = true;
    } else if (!isConnected) {
      updatedScreen = false;
    }
  }
}