#include <Arduino.h>
#include <ESP8266WiFi.h>

// Pin definitions
#define SENSOR_PIN   D2  // HW-201 OUT pin (GPIO4)
#define ALARM_LED    D1  // External Fire/Obstacle Alert LED (GPIO5)

const char* target_ssid = "VIVO V40E";
const char* target_password = "120333544";

unsigned long previousMillis = 0;
unsigned long lastLogTime = 0;
bool onboardLedState = HIGH;

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
  pinMode(ALARM_LED, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);

  digitalWrite(LED_BUILTIN, HIGH); // Onboard LED off (Active LOW)
  digitalWrite(ALARM_LED, LOW);    // Alarm LED off

  Serial.begin(115200);
  delay(100);

  // Initialize Wi-Fi in Station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // 1. Scan and print nearby networks
  scanNearbyNetworks();

  // 2. Connect to your mobile hotspot
  Serial.printf("Connecting to target network: %s\n", target_ssid);
  WiFi.begin(target_ssid, target_password);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Monitor the HW-201 IR Sensor (LOW = Triggered, HIGH = Idle)
  int sensorState = digitalRead(SENSOR_PIN);

  if (sensorState == LOW) {
    digitalWrite(ALARM_LED, HIGH); // Turn on Fire Alert LED
    if (currentMillis - lastLogTime >= 500) {
      Serial.println(">>> [ALARM TRIGGERED] Fire / IR Detected! <<<");
      lastLogTime = currentMillis;
    }
  } else {
    digitalWrite(ALARM_LED, LOW);  // Safe
  }

  // 2. Non-blocking Wi-Fi status indicator on onboard LED
  bool isConnected = (WiFi.status() == WL_CONNECTED);
  unsigned long blinkInterval = isConnected ? 1000 : 100; // 1s slow if connected, 100ms fast if searching

  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;
    onboardLedState = !onboardLedState;
    digitalWrite(LED_BUILTIN, onboardLedState);

    // Print connection success message once upon connecting
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