#include <Arduino.h>
#include <ESP8266WiFi.h>

#define IR_PIN        A0  // Analog pin for bare IR photodiode
#define ALARM_LED     D1  // GPIO5 via 220-ohm resistor

const char* target_ssid = "VIVO V40E";
const char* target_password = "120333544"; // Put your exact password

unsigned long previousMillis = 0;
unsigned long lastSensorPrint = 0;
bool ledState = false;
int baseline = 0;

void runWiFIScan() {
  Serial.println("\n--------------------------------------------------");
  Serial.println("Starting fresh 2.4 GHz Wi-Fi Scan...");
  
  // Clean radio state before scanning
  WiFi.mode(WIFI_STA);
  delay(100);

  int n = WiFi.scanNetworks(false, true); // (async = false, show_hidden = true)

  if (n == 0) {
    Serial.println("[!] No networks found. Check phone hotspot band (must be 2.4 GHz)!");
  } else {
    Serial.printf("[+] Found %d networks:\n\n", n);
    Serial.println("No. | SSID                             | RSSI     | Channel");
    Serial.println("----+----------------------------------+----------+--------");
    for (int i = 0; i < n; ++i) {
      Serial.printf("%2d  | %-32.32s | %4d dBm | %2d\n",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i),
                    WiFi.channel(i));
    }
  }
  Serial.println("--------------------------------------------------\n");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ALARM_LED, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // OFF
  digitalWrite(ALARM_LED, LOW);    // OFF

  Serial.begin(115200);
  delay(1000); // Allow USB-Serial to settle

  Serial.println("\n==========================================");
  Serial.println("   ESP8266 FULL SYSTEM HARDWARE CHECK     ");
  Serial.println("==========================================");

  // 1. Initial IR baseline reading
  baseline = analogRead(IR_PIN);
  Serial.printf("Initial IR Raw Reading on A0: %d (Range: 0 - 1023)\n", baseline);

  // 2. Wi-Fi Scan
  runWiFIScan();

  // 3. Connect to Hotspot
  Serial.printf("Connecting to Hotspot: %s\n", target_ssid);
  WiFi.begin(target_ssid, target_password);

  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter < 25) {
    delay(500);
    Serial.print(".");
    counter++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[SUCCESS] Connected to Wi-Fi!");
    Serial.print("Local IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.printf("\n[FAILED] Wi-Fi Status: %d\n", WiFi.status());
    Serial.println("(1 = SSID Not Found, 4 = Password Error, 6 = Disconnected)");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Read IR Sensor
  int rawIR = analogRead(IR_PIN);

  // Print IR sensor output every 500ms so you can verify the hardware
  if (currentMillis - lastSensorPrint >= 500) {
    Serial.printf("Live A0 IR Value: %4d | Wi-Fi: %s\n",
                  rawIR,
                  (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "SEARCHING");
    lastSensorPrint = currentMillis;
  }

  // 2. Check if IR light detected (reading jumps at least 50 above baseline)
  bool irTriggered = (rawIR > (baseline + 50)) || (rawIR > 250);

  if (irTriggered) {
    digitalWrite(ALARM_LED, HIGH);
    digitalWrite(LED_BUILTIN, LOW); // Onboard LED lit
  } else {
    // Regular blink indicator
    bool isConnected = (WiFi.status() == WL_CONNECTED);
    unsigned long blinkInterval = isConnected ? 1000 : 150;

    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH);
      digitalWrite(ALARM_LED, ledState ? HIGH : LOW);
    }
  }
}