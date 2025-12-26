#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ==================== KONFIGURASI WIFI ====================
const char* ssid = "";
const char* password = "";

// ==================== KONFIGURASI MQTT ====================
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
String clientId = "ESP32_Kelompok_2";

// ==================== PIN DEFINITIONS ====================
// SESUAIKAN PIN INI DENGAN BOARD ANDA!
// Lihat dokumentasi board atau silkscreen di PCB

// SENSOR PINS (Sesuaikan dengan board Anda)
#define DHTPIN 15           // Pin DHT sensor (contoh: GPIO15)
#define DHTTYPE DHT11       // Ubah ke DHT22 jika board pakai DHT22
#define LDR_PIN 34          // Pin LDR (biasanya ADC: GPIO32-39)
#define PIR_PIN 14          // Pin PIR sensor
#define ULTRASONIC_TRIG 5   // Pin Trigger Ultrasonic
#define ULTRASONIC_ECHO 18  // Pin Echo Ultrasonic
#define POT_PIN 35          // Pin Potensiometer (jika ada)

// AKTUATOR PINS (Sesuaikan dengan board Anda)
#define LED_RED_PIN 25      // LED Merah
#define LED_GREEN_PIN 26    // LED Hijau (jika ada)
#define LED_BLUE_PIN 27     // LED Biru (jika ada)
#define BUZZER_PIN 13       // Buzzer
#define RELAY_PIN 23        // Relay

// ==================== CONFIGURATION FLAGS ====================
// Set false jika sensor tidak ada di board Anda
#define HAS_DHT true        // Board punya sensor DHT?
#define HAS_LDR true        // Board punya LDR?
#define HAS_PIR true        // Board punya PIR?
#define HAS_ULTRASONIC true // Board punya Ultrasonic?
#define HAS_POT false       // Board punya Potensiometer?

#define HAS_LED true        // Board punya LED?
#define HAS_BUZZER true     // Board punya Buzzer?
#define HAS_RELAY true      // Board punya Relay?

// ==================== INITIALIZE OBJECTS ====================
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// ==================== VARIABLES ====================
float temperature = 0;
float humidity = 0;
float distance = 0;
int lightLevel = 0;
int potValue = 0;
bool motionDetected = false;

bool ledState = false;
bool buzzerState = false;
bool relayState = false;

unsigned long lastSensorRead = 0;
unsigned long lastPublish = 0;
const long sensorInterval = 1000;
const long publishInterval = 2000;

// ==================== TOPICS ====================
const char* TOPIC_TEMP = "Klp2/esp32/temperature";
const char* TOPIC_HUMID = "Klp2/esp32/humidity";
const char* TOPIC_DIST = "Klp2/esp32/distance";
const char* TOPIC_LIGHT = "Klp2/esp32/light";
const char* TOPIC_POT = "Klp2/esp32/potentiometer";
const char* TOPIC_MOTION = "Klp2/esp32/motion";

const char* TOPIC_LED = "Klp2/esp32/led/control";
const char* TOPIC_BUZZER = "Klp2/esp32/buzzer/control";
const char* TOPIC_RELAY = "Klp2/esp32/relay/control";

const char* TOPIC_LED_STATUS = "Klp2/esp32/led/status";
const char* TOPIC_BUZZER_STATUS = "Klp2/esp32/buzzer/status";
const char* TOPIC_RELAY_STATUS = "Klp2/esp32/relay/status";

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   ESP32 IoT Board - System Starting   ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // Initialize Sensors
  Serial.println("🔧 Initializing Sensors...");
  if (HAS_DHT) {
    dht.begin();
    Serial.println("  ✓ DHT Sensor initialized");
  }
  if (HAS_ULTRASONIC) {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    Serial.println("  ✓ Ultrasonic Sensor initialized");
  }
  if (HAS_LDR) {
    pinMode(LDR_PIN, INPUT);
    Serial.println("  ✓ LDR Sensor initialized");
  }
  if (HAS_PIR) {
    pinMode(PIR_PIN, INPUT);
    Serial.println("  ✓ PIR Sensor initialized");
  }
  if (HAS_POT) {
    pinMode(POT_PIN, INPUT);
    Serial.println("  ✓ Potentiometer initialized");
  }
  
  // Initialize Actuators
  Serial.println("\n🔧 Initializing Actuators...");
  if (HAS_LED) {
    pinMode(LED_RED_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, LOW);
    Serial.println("  ✓ LED initialized");
  }
  if (HAS_BUZZER) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("  ✓ Buzzer initialized");
  }
  if (HAS_RELAY) {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("  ✓ Relay initialized");
  }
  
  // Connect WiFi
  Serial.println("\n📡 Connecting to WiFi...");
  setup_wifi();
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║      System Ready! Monitoring...      ║");
  Serial.println("╚════════════════════════════════════════╝\n");
}

// ==================== WIFI CONNECTION ====================
void setup_wifi() {
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n  ✓ WiFi Connected!");
    Serial.print("  📍 IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("  📶 Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n  ✗ WiFi Connection Failed!");
  }
}

// ==================== MQTT RECONNECT ====================
void reconnect() {
  while (!client.connected()) {
    Serial.print("🔄 Connecting to MQTT...");
    
    String fullClientId = clientId + String(random(0xffff), HEX);
    
    if (client.connect(fullClientId.c_str())) {
      Serial.println(" ✓ Connected!");
      
      // Subscribe to control topics
      if (HAS_LED) client.subscribe(TOPIC_LED);
      if (HAS_BUZZER) client.subscribe(TOPIC_BUZZER);
      if (HAS_RELAY) client.subscribe(TOPIC_RELAY);
      
      Serial.println("  ✓ Subscribed to control topics");
      publishActuatorStatus();
      
    } else {
      Serial.print(" ✗ Failed, rc=");
      Serial.print(client.state());
      Serial.println(" | Retry in 5s...");
      delay(5000);
    }
  }
}

// ==================== MQTT CALLBACK ====================
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("📩 [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);
  
  // Control LED
  if (String(topic) == TOPIC_LED && HAS_LED) {
    if (message == "ON" || message == "1") {
      digitalWrite(LED_RED_PIN, HIGH);
      ledState = true;
      Serial.println("  💡 LED: ON");
    } else {
      digitalWrite(LED_RED_PIN, LOW);
      ledState = false;
      Serial.println("  💡 LED: OFF");
    }
  }
  
  // Control Buzzer
  if (String(topic) == TOPIC_BUZZER && HAS_BUZZER) {
    if (message == "ON" || message == "1") {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerState = true;
      Serial.println("  🔔 BUZZER: ON");
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerState = false;
      Serial.println("  🔔 BUZZER: OFF");
    }
  }
  
  // Control Relay
  if (String(topic) == TOPIC_RELAY && HAS_RELAY) {
    if (message == "ON" || message == "1") {
      digitalWrite(RELAY_PIN, HIGH);
      relayState = true;
      Serial.println("  🔌 RELAY: ON");
    } else {
      digitalWrite(RELAY_PIN, LOW);
      relayState = false;
      Serial.println("  🔌 RELAY: OFF");
    }
  }
  
  publishActuatorStatus();
}

// ==================== READ SENSORS ====================
float readDistance() {
  if (!HAS_ULTRASONIC) return 0;
  
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  if (duration == 0) return -1;
  
  return duration * 0.034 / 2;
}

int readLightLevel() {
  if (!HAS_LDR) return 0;
  
  int rawValue = analogRead(LDR_PIN);
  // Konversi ke persentase (0-100%)
  // Jika nilai tinggi = gelap, uncomment baris berikut:
  // return map(rawValue, 0, 4095, 100, 0);
  
  // Jika nilai tinggi = terang:
  return map(rawValue, 0, 4095, 0, 100);
}

bool readMotion() {
  if (!HAS_PIR) return false;
  return digitalRead(PIR_PIN) == HIGH;
}

int readPotentiometer() {
  if (!HAS_POT) return 0;
  int rawValue = analogRead(POT_PIN);
  return map(rawValue, 0, 4095, 0, 100);
}

void readAllSensors() {
  // DHT Sensor
  if (HAS_DHT) {
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    if (isnan(temperature)) temperature = -999;
    if (isnan(humidity)) humidity = -999;
  }
  
  // Ultrasonic
  if (HAS_ULTRASONIC) {
    distance = readDistance();
    if (distance < 0 || distance > 400) distance = -1;
  }
  
  // LDR
  if (HAS_LDR) {
    lightLevel = readLightLevel();
  }
  
  // PIR
  if (HAS_PIR) {
    motionDetected = readMotion();
  }
  
  // Potentiometer
  if (HAS_POT) {
    potValue = readPotentiometer();
  }
}

// ==================== PUBLISH DATA ====================
void publishSensorData() {
  char buffer[8];
  
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│        📊 SENSOR READINGS           │");
  Serial.println("├─────────────────────────────────────┤");
  
  if (HAS_DHT && temperature != -999) {
    dtostrf(temperature, 1, 2, buffer);
    client.publish(TOPIC_TEMP, buffer);
    Serial.printf("│ 🌡️  Temperature: %6.2f °C         │\n", temperature);
  }
  
  if (HAS_DHT && humidity != -999) {
    dtostrf(humidity, 1, 2, buffer);
    client.publish(TOPIC_HUMID, buffer);
    Serial.printf("│ 💧 Humidity: %9.2f %%          │\n", humidity);
  }
  
  if (HAS_ULTRASONIC && distance >= 0) {
    dtostrf(distance, 1, 2, buffer);
    client.publish(TOPIC_DIST, buffer);
    Serial.printf("│ 📏 Distance: %9.2f cm         │\n", distance);
  }
  
  if (HAS_LDR) {
    sprintf(buffer, "%d", lightLevel);
    client.publish(TOPIC_LIGHT, buffer);
    Serial.printf("│ 💡 Light Level: %5d %%          │\n", lightLevel);
  }
  
  if (HAS_PIR) {
    client.publish(TOPIC_MOTION, motionDetected ? "1" : "0");
    Serial.printf("│ 🚶 Motion: %-22s │\n", motionDetected ? "DETECTED ⚠️" : "No Motion");
  }
  
  if (HAS_POT) {
    sprintf(buffer, "%d", potValue);
    client.publish(TOPIC_POT, buffer);
    Serial.printf("│ 🎚️  Potentiometer: %4d %%         │\n", potValue);
  }
  
  Serial.println("└─────────────────────────────────────┘\n");
}

void publishActuatorStatus() {
  if (HAS_LED) client.publish(TOPIC_LED_STATUS, ledState ? "ON" : "OFF");
  if (HAS_BUZZER) client.publish(TOPIC_BUZZER_STATUS, buzzerState ? "ON" : "OFF");
  if (HAS_RELAY) client.publish(TOPIC_RELAY_STATUS, relayState ? "ON" : "OFF");
}

// ==================== MAIN LOOP ====================
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastSensorRead >= sensorInterval) {
    lastSensorRead = currentMillis;
    readAllSensors();
  }
  
  if (currentMillis - lastPublish >= publishInterval) {
    lastPublish = currentMillis;
    publishSensorData();
  }
}

// ==================== HELPER: PRINT BOARD CONFIG ====================
void printBoardConfig() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║         BOARD CONFIGURATION           ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ SENSORS:                              ║");
  if (HAS_DHT) Serial.printf("║   ✓ DHT%-2s on GPIO %-2d              ║\n", DHTTYPE == DHT11 ? "11" : "22", DHTPIN);
  if (HAS_ULTRASONIC) Serial.printf("║   ✓ Ultrasonic (T:%d, E:%d)         ║\n", ULTRASONIC_TRIG, ULTRASONIC_ECHO);
  if (HAS_LDR) Serial.printf("║   ✓ LDR on GPIO %-2d                  ║\n", LDR_PIN);
  if (HAS_PIR) Serial.printf("║   ✓ PIR on GPIO %-2d                  ║\n", PIR_PIN);
  if (HAS_POT) Serial.printf("║   ✓ Potentiometer on GPIO %-2d        ║\n", POT_PIN);
  Serial.println("║                                       ║");
  Serial.println("║ ACTUATORS:                            ║");
  if (HAS_LED) Serial.printf("║   ✓ LED on GPIO %-2d                  ║\n", LED_RED_PIN);
  if (HAS_BUZZER) Serial.printf("║   ✓ Buzzer on GPIO %-2d               ║\n", BUZZER_PIN);
  if (HAS_RELAY) Serial.printf("║   ✓ Relay on GPIO %-2d                ║\n", RELAY_PIN);
  Serial.println("╚════════════════════════════════════════╝\n");
}
