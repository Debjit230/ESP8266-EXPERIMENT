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

// Pin Mappings
#define OLED_SDA      D3  // GPIO0
#define OLED_SCL      D4  // GPIO2
#define IR_RECV_PIN   D2  // TSOP OUT (GPIO4)
#define AUDIO_ANA_PIN A0  // LM358 Pin 1 (OUT1)
#define EXTERNAL_LED  D1  // Indicator LED via 220-ohm resistor

// Sound & Clap Thresholds
const int CLAP_DELTA = 180; // Spike required above/below baseline to trigger clap

IRrecv irrecv(IR_RECV_PIN, 1024, 50, true);
decode_results results;

const char* target_ssid = "Airfiber-3rdFloorBachelor";
const char* target_password = "Airfiber-3rdfloor";

bool ledToggled = false;
unsigned long lastClapTime = 0;
unsigned long lastDisplayUpdate = 0;
String lastIRCode = "None";

// IR LED blink timing variables
bool irBlinkActive = false;
unsigned long irBlinkStart = 0;
const unsigned long IR_BLINK_DURATION = 80; // Duration of flash in milliseconds

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

  bootIndicator();

  Serial.begin(115200);
  delay(200);

  // Initialize OLED on D3 & D4
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  irrecv.enableIRIn();

  // Connect to Wi-Fi
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(target_ssid, target_password);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Read Audio Waveform from LM358 Pin 1
  int audioSample = analogRead(AUDIO_ANA_PIN);
  int soundAmplitude = abs(audioSample - 512); // Distance from 1.65V center

  // Dynamic sound bar width for OLED
  int soundBarWidth = map(soundAmplitude, 0, 350, 0, 120);
  soundBarWidth = constrain(soundBarWidth, 0, 120);

  // 2. Software Clap Detection (toggles base state)
  if (soundAmplitude > CLAP_DELTA) {
    if (currentMillis - lastClapTime > 300) { // 300ms debounce
      ledToggled = !ledToggled;
      Serial.printf(">>> [CLAP TRIGGERED] A0 Peak: %d | Base State: %s <<<\n",
                    audioSample, ledToggled ? "ON" : "OFF");
      lastClapTime = currentMillis;
    }
  }

  // 3. IR Remote Signal Capture & Trigger Flash
  if (irrecv.decode(&results)) {
    lastIRCode = "0x" + uint64ToString(results.value, HEX);
    
    // Trigger non-blocking visual feedback blink
    irBlinkActive = true;
    irBlinkStart = currentMillis;

    Serial.printf(">>> [IR CAPTURED] Code: %s <<<\n", lastIRCode.c_str());
    irrecv.resume();
  }

  // 4. Manage LED State (IR flash overrides or inverts base clap state)
  if (irBlinkActive) {
    if (currentMillis - irBlinkStart < IR_BLINK_DURATION) {
      // Invert state during the blink pulse so it is visible whether clap LED is ON or OFF
      digitalWrite(EXTERNAL_LED, ledToggled ? LOW : HIGH);
    } else {
      irBlinkActive = false;
      digitalWrite(EXTERNAL_LED, ledToggled ? HIGH : LOW);
    }
  } else {
    digitalWrite(EXTERNAL_LED, ledToggled ? HIGH : LOW);
  }

  // 5. OLED Display Refresh (~25 FPS)
  if (currentMillis - lastDisplayUpdate >= 40) {
    lastDisplayUpdate = currentMillis;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // Wi-Fi Status Banner
    display.setCursor(0, 0);
    display.printf("WiFi: %s", (WiFi.status() == WL_CONNECTED) ? "ONLINE" : "SEARCH");

    // IR Code
    display.setCursor(0, 14);
    display.printf("IR  : %s", lastIRCode.c_str());

    // Sound Stats
    display.setCursor(0, 26);
    display.printf("Raw A0 : %d", audioSample);
    display.setCursor(0, 38);
    display.printf("LED (Clap): %s", ledToggled ? "ON" : "OFF");

    // Live Audio VU Bar
    display.drawRect(0, 50, 124, 12, SSD1306_WHITE);
    display.fillRect(2, 52, soundBarWidth, 8, SSD1306_WHITE);

    display.display();
  }
}