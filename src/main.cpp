#include <Arduino.h>
#include <ESP8266WiFi.h>

const char* target_ssid = "VIVO V40E";
const char* target_password = "120333544";

unsigned long previousMillis = 0;
bool ledState = HIGH;

// Helper to convert encryption type enum to readable text
String getEncryptionType(uint8_t encType) {
  switch (encType) {
    case ENC_TYPE_NONE: return "Open";
    case ENC_TYPE_WEP:  return "WEP";
    case ENC_TYPE_TKIP: return "WPA/PSK";
    case ENC_TYPE_CCMP: return "WPA2/PSK";
    case ENC_TYPE_AUTO: return "WPA/WPA2/Auto";
    default:            return "Unknown";
  }
}

void scanNearbyNetworks() {
  Serial.println("\n--------------------------------------------------");
  Serial.println("Scanning nearby 2.4 GHz Wi-Fi networks...");
  
  // WiFi.scanNetworks returns the total number of APs found
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("No Wi-Fi networks found.");
  } else {
    Serial.printf("Found %d networks:\n\n", n);
    Serial.println("No. | SSID                             | RSSI     | Ch | Security");
    Serial.println("----+----------------------------------+----------+----+-----------");

    for (int i = 0; i < n; ++i) {
      Serial.printf("%2d  | %-32.32s | %4d dBm | %2d | %s\n",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i),
                    WiFi.channel(i),
                    getEncryptionType(WiFi.encryptionType(i)).c_str());
    }
  }
  Serial.println("--------------------------------------------------\n");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Active LOW -> OFF

  Serial.begin(115200);
  delay(100);

  // Put Wi-Fi in Station mode and disconnect from previous sessions
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // 1. Scan and print nearby networks
  scanNearbyNetworks();

  // 2. Begin connecting to your target AP
  Serial.printf("Connecting to target network: %s\n", target_ssid);
  WiFi.begin(target_ssid, target_password);
}

void loop() {
  unsigned long currentMillis = millis();

  // Dynamic LED blink state: 1000ms if connected, 100ms if searching/disconnected
  bool isConnected = (WiFi.status() == WL_CONNECTED);
  unsigned long blinkInterval = isConnected ? 1000 : 100;

  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);

    // Print connection success once
    static bool loggedConnection = false;
    if (isConnected && !loggedConnection) {
      Serial.println("\n>>> Wi-Fi Connected Successfully! <<<");
      Serial.print("Assigned IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Target Signal: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm\n");
      loggedConnection = true;
    } else if (!isConnected) {
      loggedConnection = false;
    }
  }
}