#include <Arduino.h>
#include <ESP8266WiFi.h>

#define EXTERNAL_LED D1  // GPIO5 connected via 220 ohm resistor

const char* target_ssid = "VIVO V40E";
const char* target_password = "YOUR_PASSWORD"; // Put your password here

unsigned long previousMillis = 0;
bool ledState = false;

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

// Visual boot indicator across both LEDs
void bootIndicator() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, LOW);   // Onboard LED ON (active LOW)
    digitalWrite(EXTERNAL_LED, HIGH); // External LED ON (active HIGH)
    delay(80);
    digitalWrite(LED_BUILTIN, HIGH);  // Onboard LED OFF
    digitalWrite(EXTERNAL_LED, LOW);  // External LED OFF
    delay(80);
  }
}

void scanNearbyNetworks() {
  Serial.println("\n--------------------------------------------------");
  Serial.println("Scanning nearby 2.4 GHz Wi-Fi networks...");

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
  pinMode(EXTERNAL_LED, OUTPUT);

  // Initial startup flash
  bootIndicator();

  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- ESP8266 System Starting ---");

  // Wi-Fi initialization
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Scan and display nearby networks
  scanNearbyNetworks();

  Serial.printf("Connecting to target network: %s\n", target_ssid);
  WiFi.begin(target_ssid, target_password);
}

void loop() {
  unsigned long currentMillis = millis();

  bool isConnected = (WiFi.status() == WL_CONNECTED);
  unsigned long blinkInterval = isConnected ? 1000 : 100; // 1s slow if connected, 100ms fast if connecting

  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;
    ledState = !ledState;

    // Toggle onboard LED (active LOW)
    digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH);

    // Toggle external LED on D1 (active HIGH)
    digitalWrite(EXTERNAL_LED, ledState ? HIGH : LOW);

    // Print Wi-Fi connection info once
    static bool loggedConnection = false;
    if (isConnected && !loggedConnection) {
      Serial.println("\n>>> Wi-Fi Connected Successfully! <<<");
      Serial.print("Assigned IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Signal Strength: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm\n");
      loggedConnection = true;
    } else if (!isConnected) {
      loggedConnection = false;
    }
  }
}