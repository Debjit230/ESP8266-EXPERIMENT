#include <Arduino.h>
#include <ESP8266WiFi.h>

#define IR_PIN        A0  // Reverse-bias bare IR photodiode junction
#define ALARM_LED     D1  // External LED on D1 (via 220 ohm resistor)

// Adjust based on ambient room light readings from Serial Monitor
const int FIRE_THRESHOLD = 200; 

const char* target_ssid = "VIVO V40E";
const char* target_password = "120333544"; // Insert your hotspot password

unsigned long previousMillis = 0;
unsigned long lastSerialPrint = 0;
bool ledState = false;

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

void bootIndicator() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, LOW);   // Onboard ON (active LOW)
    digitalWrite(ALARM_LED, HIGH);    // External ON (active HIGH)
    delay(80);
    digitalWrite(LED_BUILTIN, HIGH);  // Onboard OFF
    digitalWrite(ALARM_LED, LOW);     // External OFF
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
  pinMode(ALARM_LED, OUTPUT);

  // Run startup flash animation on both LEDs
  bootIndicator();

  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- ESP8266 Fire Alarm & Wi-Fi Node Online ---");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Scan and print local networks
  scanNearbyNetworks();

  Serial.printf("Connecting to: %s\n", target_ssid);
  WiFi.begin(target_ssid, target_password);
}

void loop() {
  unsigned long currentMillis = millis();

  // Read analog value from the IR divider circuit
  int irLevel = analogRead(IR_PIN);
  bool fireDetected = (irLevel > FIRE_THRESHOLD);

  if (fireDetected) {
    // Fire Hazard: Turn both LEDs solidly ON
    digitalWrite(ALARM_LED, HIGH);
    digitalWrite(LED_BUILTIN, LOW);

    if (currentMillis - lastSerialPrint >= 400) {
      Serial.printf(">>> [FIRE HAZARD DETECTED!] Analog IR: %d (Threshold: %d) <<<\n", irLevel, FIRE_THRESHOLD);
      lastSerialPrint = currentMillis;
    }
  } else {
    // Normal operation: LEDs flash synchronously to reflect Wi-Fi status
    bool isConnected = (WiFi.status() == WL_CONNECTED);
    unsigned long blinkInterval = isConnected ? 1000 : 100; // 1000ms connected, 100ms searching

    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      ledState = !ledState;

      digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH);
      digitalWrite(ALARM_LED, ledState ? HIGH : LOW);

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

    if (currentMillis - lastSerialPrint >= 1500) {
      Serial.printf("Status: Safe | Ambient IR Level: %d\n", irLevel);
      lastSerialPrint = currentMillis;
    }
  }
}