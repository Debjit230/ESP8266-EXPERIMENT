#include <Arduino.h>
#include <ESP8266WiFi.h>

#define IR_PIN        A0  // Reverse-bias bare IR photodiode divider
#define ALARM_LED     D1  // External visual indicator LED

const char* target_ssid = "VIVO V40E";
const char* target_password = "120333544";

// Dynamic threshold parameters
int ambientBaseline = 0;
int irThreshold = 150;
const int SENSITIVITY_MARGIN = 60; // Offset above ambient light to trigger detection

unsigned long previousMillis = 0;
unsigned long lastLogTime = 0;
bool ledState = false;

void bootIndicator() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(ALARM_LED, HIGH);
    delay(70);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(ALARM_LED, LOW);
    delay(70);
  }
}

// Samples room light for 1 second to calibrate baseline
void calibrateAmbientLight() {
  long sum = 0;
  const int samples = 50;

  Serial.println("Calibrating ambient IR baseline... keep sensor still.");
  for (int i = 0; i < samples; i++) {
    sum += analogRead(IR_PIN);
    delay(20);
  }

  ambientBaseline = sum / samples;
  irThreshold = ambientBaseline + SENSITIVITY_MARGIN;

  Serial.printf("Calibration Complete -> Baseline: %d | Trigger Threshold: %d\n\n",
                ambientBaseline, irThreshold);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ALARM_LED, OUTPUT);

  bootIndicator();

  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- ESP8266 IR Light Detector Online ---");

  // Calibrate photodiode to room lighting
  calibrateAmbientLight();

  // Wi-Fi Configuration
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);

  Serial.printf("Connecting to Wi-Fi: %s\n", target_ssid);
  WiFi.begin(target_ssid, target_password);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Read IR light intensity
  int irLevel = analogRead(IR_PIN);
  bool irDetected = (irLevel > irThreshold);

  if (irDetected) {
    // IR Source Detected: Lock both LEDs ON
    digitalWrite(ALARM_LED, HIGH);
    digitalWrite(LED_BUILTIN, LOW); // NodeMCU onboard LED is active LOW

    if (currentMillis - lastLogTime >= 200) {
      Serial.printf(">>> [IR LIGHT DETECTED] Level: %d | Delta: +%d <<<\n",
                    irLevel, (irLevel - ambientBaseline));
      lastLogTime = currentMillis;
    }
  } else {
    // 2. Wi-Fi Status Pulse when idle
    bool isConnected = (WiFi.status() == WL_CONNECTED);
    unsigned long blinkInterval = isConnected ? 1000 : 100;

    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      ledState = !ledState;

      digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH);
      digitalWrite(ALARM_LED, ledState ? HIGH : LOW);
    }

    if (currentMillis - lastLogTime >= 1500) {
      Serial.printf("Monitoring... Ambient: %d | Live Level: %d | Status: %s\n",
                    ambientBaseline, irLevel, isConnected ? "Connected" : "Connecting");
      lastLogTime = currentMillis;
    }
  }
}