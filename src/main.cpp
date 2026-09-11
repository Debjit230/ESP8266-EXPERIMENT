#include <Arduino.h>
#include <ESP8266WiFi.h>

const char* ssid = "VIVO V40E";
const char* password = "120333544";

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n--- ESP8266 Wi-Fi Setup ---");

  // Explicitly set ESP8266 as station (client) mode
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi Connected successfully!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal Strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void loop() {
  // Stay connected or perform network tasks here
}