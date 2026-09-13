#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin Mappings
#define OLED_SDA     D3  // GPIO0
#define OLED_SCL     D4  // GPIO2
#define IR_RECV_PIN  D2  // TSOP OUT Pin (GPIO4)
#define EXTERNAL_LED D1  // Visual indicator LED

const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50; // Milliseconds of silence to define message end

IRrecv irrecv(IR_RECV_PIN, kCaptureBufferSize, kTimeout, true);
decode_results results;

// Wi-Fi Credentials
const char* target_ssid = "Airfiber-3rdFloorBachelor";
const char* target_password = "Airfiber-3rdfloor"; // Enter your hotspot password

unsigned long previousMillis = 0;
unsigned long lastWifiCheck = 0;
bool ledState = false;

// Function to update OLED interface
void updateOLED(const String& protocol, const String& hexCode, int bits, const String& wifiStatus) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header Banner
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("--- IR DECODER ---");

  // Protocol Info
  display.setCursor(0, 14);
  display.print("Proto: ");
  display.println(protocol);

  // Hex Code Value
  display.setCursor(0, 26);
  display.print("Code : ");
  display.setTextSize(1);
  display.println(hexCode);

  // Bit Depth
  display.setCursor(0, 38);
  display.printf("Bits : %d-bit\n", bits);

  // Wi-Fi Status Bar
  display.setCursor(0, 52);
  display.printf("WiFi : %s", wifiStatus.c_str());

  display.display();
}

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

  // 4 Fast Blinks on Boot
  bootIndicator();

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP8266 IR Remote Scanner & Decoder ===");

  // Initialize I2C OLED on D3 & D4
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[!] SSD1306 allocation failed. Check D3/D4 connections!"));
  } else {
    display.clearDisplay();
    display.display();
  }

  updateOLED("Ready", "Press Remote", 0, "Connecting...");

  // Start IR Receiver
  irrecv.enableIRIn();
  Serial.printf("IR Receiver listening on Pin D2 (GPIO%d)...\n", IR_RECV_PIN);

  // Initialize Wi-Fi
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(target_ssid, target_password);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Check for incoming IR Remote Signal
  if (irrecv.decode(&results)) {
    // Pulse external LED quickly to confirm capture
    digitalWrite(EXTERNAL_LED, HIGH);

    String protocolStr = typeToString(results.decode_type);
    String hexStr = "0x" + uint64ToString(results.value, HEX);

    // Print detailed remote signature to Serial Monitor
    Serial.println("\n--- IR SIGNAL DETECTED ---");
    Serial.printf("Protocol : %s\n", protocolStr.c_str());
    Serial.printf("Hex Code : %s\n", hexStr.c_str());
    Serial.printf("Bit Depth: %d bits\n", results.bits);
    Serial.println("Carrier  : Standard 38kHz Bandpass Filtered");

    // Display captured code directly on the OLED
    String wifiStatus = (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "OFFLINE";
    updateOLED(protocolStr, hexStr, results.bits, wifiStatus);

    delay(40);
    digitalWrite(EXTERNAL_LED, LOW);

    // Ready receiver for next signal
    irrecv.resume();
  }

  // 2. Wi-Fi Reconnect Watchdog (every 5 seconds)
  if (currentMillis - lastWifiCheck >= 5000) {
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
  }

  // 3. Heartbeat LED blink when idle (1000ms if connected, 150ms if reconnecting)
  bool isConnected = (WiFi.status() == WL_CONNECTED);
  unsigned long blinkInterval = isConnected ? 1000 : 150;

  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(EXTERNAL_LED, ledState ? HIGH : LOW);
  }
}