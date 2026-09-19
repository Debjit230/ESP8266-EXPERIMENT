#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define OLED_SDA     D3  // GPIO0
#define OLED_SCL     D4  // GPIO2
#define IR_RECV_PIN  D2  // TSOP OUT Pin (GPIO4)
#define EXTERNAL_LED D1  // Visual indicator LED

const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50; // Timeout in ms

// Set save_buffer = true to analyze signals properly
IRrecv irrecv(IR_RECV_PIN, kCaptureBufferSize, kTimeout, true);
decode_results results;

const char* target_ssid = "Airfiber-3rdFloorBachelor";
const char* target_password = "Airfiber-3rdfloor";

unsigned long previousMillis = 0;
unsigned long lastWifiCheck = 0;
bool ledState = false;

// Non-blocking IR blink state
bool irBlinkActive = false;
unsigned long irBlinkStart = 0;
const unsigned long IR_BLINK_DURATION = 80;

void updateOLED(const String& protocol, const String& hexCode, int bits, const String& wifiStatus) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("--- IR DECODER ---");

  display.setCursor(0, 14);
  display.print("Proto: ");
  display.println(protocol);

  display.setCursor(0, 26);
  display.print("Code : ");
  display.println(hexCode);

  display.setCursor(0, 38);
  display.printf("Bits : %d-bit\n", bits);

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

  // Enable internal pull-up on D2 to prevent floating pin noise
  pinMode(IR_RECV_PIN, INPUT_PULLUP);

  bootIndicator();

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP8266 IR Remote Scanner & Decoder ===");

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[!] SSD1306 allocation failed."));
  } else {
    display.clearDisplay();
    display.display();
  }

  updateOLED("Ready", "Press Remote", 0, "Connecting...");

  // Start IR Receiver with noise threshold
  irrecv.setUnknownThreshold(12); // Ignore short bursts < 12 ticks
  irrecv.enableIRIn();

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(target_ssid, target_password);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Process Incoming IR Signal
  if (irrecv.decode(&results)) {
    // FILTER: Ignore UNKNOWN noise and invalid 0-bit bursts
    bool isValidSignal = (results.decode_type != decode_type_t::UNKNOWN) && 
                         (results.bits >= 8) && 
                         (results.value != 0);

    if (isValidSignal) {
      String protocolStr = typeToString(results.decode_type);
      String hexStr = "0x" + uint64ToString(results.value, HEX);

      Serial.println("\n--- VALID IR SIGNAL DETECTED ---");
      Serial.printf("Protocol : %s\n", protocolStr.c_str());
      Serial.printf("Hex Code : %s\n", hexStr.c_str());
      Serial.printf("Bit Depth: %d bits\n", results.bits);

      String wifiStatus = (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "OFFLINE";
      updateOLED(protocolStr, hexStr, results.bits, wifiStatus);

      // Trigger LED blink feedback
      irBlinkActive = true;
      irBlinkStart = currentMillis;
    }

    // Always resume receiver to clear buffer
    irrecv.resume();
  }

  // 2. Non-blocking IR Flash Management
  if (irBlinkActive) {
    if (currentMillis - irBlinkStart < IR_BLINK_DURATION) {
      digitalWrite(EXTERNAL_LED, HIGH);
    } else {
      irBlinkActive = false;
      digitalWrite(EXTERNAL_LED, LOW);
    }
  }

  // 3. Wi-Fi Reconnect Watchdog (every 5 seconds)
  if (currentMillis - lastWifiCheck >= 5000) {
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
  }

  // 4. Idle Heartbeat Blink (only when IR flash is not active)
  if (!irBlinkActive) {
    bool isConnected = (WiFi.status() == WL_CONNECTED);
    unsigned long blinkInterval = isConnected ? 1000 : 150;

    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(EXTERNAL_LED, ledState ? HIGH : LOW);
    }
  }
}