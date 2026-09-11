#include <Arduino.h>
#include <ESP8266WiFi.h>

const char* ssid = "VIVO V40E";
const char* password = "120333544";

unsigned long previousMillis = 0;
bool ledState = HIGH;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Off by default (active LOW on NodeMCU)

  Serial.begin(115200);
  delay(100);

  Serial.println("\n--- ESP8266 Wi-Fi with Status LED ---");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

void loop() {
  unsigned long currentMillis = millis();

  // Determine interval based on Wi-Fi connection state
  bool isConnected = (WiFi.status() == WL_CONNECTED);
  unsigned long blinkInterval = isConnected ? 1000 : 100; // 1000ms slow, 100ms fast

  // Non-blocking LED toggle
  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);

    // Print connection confirmation only once right when connected
    static bool loggedConnection = false;
    if (isConnected && !loggedConnection) {
      Serial.println("\nWi-Fi Connected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      loggedConnection = true;
    } else if (!isConnected) {
      loggedConnection = false; // Reset flag if signal drops
    }
  }
}