#include <Arduino.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("\n--- ESP8266 Online ---");
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);   // NodeMCU onboard LED turns ON (active LOW)
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);  // NodeMCU onboard LED turns OFF
  delay(1000);
  Serial.println("Tick... LED toggled");
}