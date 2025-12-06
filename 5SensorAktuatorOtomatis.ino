#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ==================== KONFIGURASI WIFI ====================
const char* ssid = "TD";
const char* password = "seterahgue";

// ==================== KONFIGURASI MQTT ====================
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
String clientId = "ESP32_Kelompok_2";

// ==================== PIN DEFINITIONS ====================
#define DHTPIN 15           
#define DHTTYPE DHT11       
#define LDR_PIN 34          
#define PIR_PIN 14          
#define ULTRASONIC_TRIG 5   
#define ULTRASONIC_ECHO 18  

#define LED_RED_PIN 25      
#define BUZZER_PIN 13       
#define RELAY_PIN 23        

// ==================== CONFIGURATION ====================
#define HAS_DHT true        
#define HAS_LDR true        
#define HAS_PIR true        
#define HAS_ULTRASONIC true 

#define HAS_LED true        
#define HAS_BUZZER true     
#define HAS_RELAY true      

// ==================== AUTOMATION SETTINGS ====================
// Set true untuk enable automation, false untuk pure manual
#define AUTO_LIGHTING true       // Auto LED saat gelap
#define AUTO_SECURITY true       // Auto alarm saat motion
#define AUTO_COOLING true        // Auto relay (fan/AC) saat panas
#define AUTO_PROXIMITY true      // Auto buzzer saat jarak dekat

// Threshold values - SESUAIKAN dengan kebutuhan
#define LIGHT_THRESHOLD 30       // Cahaya < 30% = gelap
#define TEMP_THRESHOLD 30.0      // Suhu > 30°C = panas
#define DISTANCE_THRESHOLD 20.0  // Jarak < 20cm = terlalu dekat

// Manual override - jika true, manual control dari dashboard aktif
bool manualOverride = false;

// ==================== OBJECTS ====================
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// ==================== VARIABLES ====================
float temperature = 0, humidity = 0, distance = 0;
int lightLevel = 0;
bool motionDetected = false;

bool ledState = false, buzzerState = false, relayState = false;
bool ledAuto = false, buzzerAuto = false, relayAuto = false;

unsigned long lastSensorRead = 0;
unsigned long lastPublish = 0;
unsigned long lastAutomation = 0;
const long sensorInterval = 500;   
const long publishInterval = 2000;
const long automationInterval = 1000;  // Check automation setiap 1 detik

// ==================== TOPICS ====================
const char* TOPIC_TEMP = "Klp2/esp32/temperature";
const char* TOPIC_HUMID = "Klp2/esp32/humidity";
const char* TOPIC_DIST = "Klp2/esp32/distance";
const char* TOPIC_LIGHT = "Klp2/esp32/light";
const char* TOPIC_MOTION = "Klp2/esp32/motion";

const char* TOPIC_LED = "Klp2/esp32/led/control";
const char* TOPIC_BUZZER = "Klp2/esp32/buzzer/control";
const char* TOPIC_RELAY = "Klp2/esp32/relay/control";

const char* TOPIC_LED_STATUS = "Klp2/esp32/led/status";
const char* TOPIC_BUZZER_STATUS = "Klp2/esp32/buzzer/status";
const char* TOPIC_RELAY_STATUS = "Klp2/esp32/relay/status";

const char* TOPIC_MODE = "Klp2/esp32/mode/control";  // AUTO/MANUAL

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  printBanner();
  initializeSensors();
  initializeActuators();
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║      🤖 SMART AUTOMATION MODE ACTIVE! 🤖          ║");
  Serial.println("║                                                    ║");
  Serial.println("║  Sistem akan otomatis mengontrol aktuator         ║");
  Serial.println("║  berdasarkan kondisi sensor:                      ║");
  Serial.println("║                                                    ║");
  if (AUTO_LIGHTING) {
  Serial.println("║  🌙 Cahaya < 30% → LED ON (Smart Lighting)        ║");
  }
  if (AUTO_SECURITY) {
  Serial.println("║  🚶 Motion Detected → Alarm ON (Security)         ║");
  }
  if (AUTO_COOLING) {
  Serial.println("║  🌡️  Suhu > 30°C → Relay ON (Cooling)             ║");
  }
  if (AUTO_PROXIMITY) {
  Serial.println("║  📏 Jarak < 20cm → Buzzer ON (Proximity Alert)   ║");
  }
  Serial.println("║                                                    ║");
  Serial.println("║  📱 Tetap bisa kontrol manual dari dashboard!     ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");
}

void printBanner() {
  Serial.println("\n╔═════════════════════════════════════════════════╗");
  Serial.println("║     ESP32 IoT - SMART AUTOMATION SYSTEM        ║");
  Serial.println("║     Sensor-Based Intelligent Control           ║");
  Serial.println("╚═════════════════════════════════════════════════╝\n");
}

void initializeSensors() {
  Serial.println("🔧 Initializing Sensors...");
  if (HAS_DHT) {
    dht.begin();
    Serial.println("  ✓ DHT Sensor");
  }
  if (HAS_ULTRASONIC) {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    Serial.println("  ✓ Ultrasonic Sensor");
  }
  if (HAS_LDR) {
    pinMode(LDR_PIN, INPUT);
    Serial.println("  ✓ LDR Sensor");
  }
  if (HAS_PIR) {
    pinMode(PIR_PIN, INPUT);
    Serial.println("  ✓ PIR Sensor");
  }
}

void initializeActuators() {
  Serial.println("\n🔧 Initializing Actuators...");
  if (HAS_LED) {
    pinMode(LED_RED_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, LOW);
    Serial.println("  ✓ LED (Smart Lighting)");
  }
  if (HAS_BUZZER) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("  ✓ Buzzer (Alarm System)");
  }
  if (HAS_RELAY) {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("  ✓ Relay (Cooling/Heating)");
  }
}

// ==================== WIFI ====================
void setup_wifi() {
  Serial.print("\n📡 Connecting WiFi");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" ✓");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
  }
}

// ==================== MQTT ====================
void reconnect() {
  while (!client.connected()) {
    Serial.print("🔄 MQTT...");
    
    String fullClientId = clientId + String(random(0xffff), HEX);
    
    if (client.connect(fullClientId.c_str())) {
      Serial.println(" ✓");
      
      client.subscribe(TOPIC_LED);
      client.subscribe(TOPIC_BUZZER);
      client.subscribe(TOPIC_RELAY);
      client.subscribe(TOPIC_MODE);
      
      publishActuatorStatus();
      
    } else {
      Serial.print(" ✗ rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("\n📩 [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);
  
  // Mode control: AUTO/MANUAL
  if (String(topic) == TOPIC_MODE) {
    if (message == "MANUAL") {
      manualOverride = true;
      Serial.println("  ⚙️  MODE: MANUAL CONTROL");
    } else if (message == "AUTO") {
      manualOverride = false;
      Serial.println("  🤖 MODE: AUTO CONTROL");
    }
    return;
  }
  
  // Manual control (hanya aktif jika manualOverride = true)
  if (manualOverride) {
    if (String(topic) == TOPIC_LED) {
      ledState = (message == "ON" || message == "1");
      digitalWrite(LED_RED_PIN, ledState ? HIGH : LOW);
      Serial.printf("  💡 LED Manual: %s\n", ledState ? "ON" : "OFF");
    }
    
    if (String(topic) == TOPIC_BUZZER) {
      buzzerState = (message == "ON" || message == "1");
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
      Serial.printf("  🔔 Buzzer Manual: %s\n", buzzerState ? "ON" : "OFF");
    }
    
    if (String(topic) == TOPIC_RELAY) {
      relayState = (message == "ON" || message == "1");
      digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
      Serial.printf("  🔌 Relay Manual: %s\n", relayState ? "ON" : "OFF");
    }
    
    publishActuatorStatus();
  } else {
    Serial.println("  ⚠️  Manual control disabled - System in AUTO mode");
    Serial.println("     Send 'MANUAL' to topic 'mode/control' to enable");
  }
}

// ==================== SENSOR READING ====================
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
  if (!HAS_LDR) return 50;
  int rawValue = analogRead(LDR_PIN);
  return map(rawValue, 0, 4095, 0, 100);
}

bool readMotion() {
  if (!HAS_PIR) return false;
  return digitalRead(PIR_PIN) == HIGH;
}

void readAllSensors() {
  if (HAS_DHT) {
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    if (isnan(temperature)) temperature = 25.0;
    if (isnan(humidity)) humidity = 60.0;
  }
  
  if (HAS_ULTRASONIC) {
    distance = readDistance();
    if (distance < 0 || distance > 400) distance = 100.0;
  }
  
  if (HAS_LDR) {
    lightLevel = readLightLevel();
  }
  
  if (HAS_PIR) {
    motionDetected = readMotion();
  }
}

// ==================== AUTOMATION LOGIC ====================
void runAutomation() {
  if (manualOverride) {
    return;  // Skip automation jika mode manual
  }
  
  bool ledPrev = ledAuto;
  bool buzzerPrev = buzzerAuto;
  bool relayPrev = relayAuto;
  
  // SCENARIO 1: Smart Lighting - LED ON saat gelap
  if (AUTO_LIGHTING && HAS_LDR) {
    if (lightLevel < LIGHT_THRESHOLD) {
      ledAuto = true;
    } else {
      ledAuto = false;
    }
  }
  
  // SCENARIO 2: Security Alarm - Buzzer + LED saat motion
  if (AUTO_SECURITY && HAS_PIR) {
    if (motionDetected) {
      buzzerAuto = true;
      ledAuto = true;  // LED juga nyala sebagai indikator
    } else {
      buzzerAuto = false;
      // LED tetap ngikut smart lighting
    }
  }
  
  // SCENARIO 3: Auto Cooling - Relay ON saat panas
  if (AUTO_COOLING && HAS_DHT) {
    if (temperature > TEMP_THRESHOLD) {
      relayAuto = true;
    } else {
      relayAuto = false;
    }
  }
  
  // SCENARIO 4: Proximity Alert - Buzzer saat jarak dekat
  if (AUTO_PROXIMITY && HAS_ULTRASONIC) {
    if (distance > 0 && distance < DISTANCE_THRESHOLD) {
      buzzerAuto = true;
    } else if (!motionDetected) {  // Jika tidak ada motion, buzzer off
      buzzerAuto = false;
    }
  }
  
  // Apply automation ke hardware
  ledState = ledAuto;
  buzzerState = buzzerAuto;
  relayState = relayAuto;
  
  digitalWrite(LED_RED_PIN, ledState ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
  digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
  
  // Log perubahan automation
  if (ledState != ledPrev || buzzerState != buzzerPrev || relayState != relayPrev) {
    Serial.println("\n🤖 ═══ AUTOMATION TRIGGERED ═══");
    
    if (ledState != ledPrev) {
      Serial.printf("   💡 LED: %s ", ledState ? "ON" : "OFF");
      if (lightLevel < LIGHT_THRESHOLD) {
        Serial.println("(Cahaya rendah)");
      } else if (motionDetected) {
        Serial.println("(Motion detected)");
      } else {
        Serial.println();
      }
    }
    
    if (buzzerState != buzzerPrev) {
      Serial.printf("   🔔 Buzzer: %s ", buzzerState ? "ON" : "OFF");
      if (motionDetected) {
        Serial.println("(Security Alert!)");
      } else if (distance < DISTANCE_THRESHOLD) {
        Serial.println("(Proximity Alert!)");
      } else {
        Serial.println();
      }
    }
    
    if (relayState != relayPrev) {
      Serial.printf("   🔌 Relay: %s ", relayState ? "ON" : "OFF");
      if (temperature > TEMP_THRESHOLD) {
        Serial.printf("(Suhu: %.1f°C)\n", temperature);
      } else {
        Serial.println();
      }
    }
    
    publishActuatorStatus();
  }
}

// ==================== PUBLISH ====================
void publishSensorData() {
  char buffer[8];
  
  Serial.println("┌──────────────────────────────────────────────┐");
  Serial.printf("│ 📊 SENSOR DATA [%s Mode]%s│\n", 
                manualOverride ? "MANUAL" : "AUTO  ",
                manualOverride ? "              " : "               ");
  Serial.println("├──────────────────────────────────────────────┤");
  
  if (HAS_DHT) {
    dtostrf(temperature, 1, 2, buffer);
    client.publish(TOPIC_TEMP, buffer);
    Serial.printf("│ 🌡️  Suhu:         %6.2f °C ", temperature);
    if (temperature > TEMP_THRESHOLD) Serial.print("⚠️ PANAS");
    Serial.println();
    
    dtostrf(humidity, 1, 2, buffer);
    client.publish(TOPIC_HUMID, buffer);
    Serial.printf("│ 💧 Kelembaban:   %6.2f %%                │\n", humidity);
  }
  
  if (HAS_ULTRASONIC) {
    dtostrf(distance, 1, 2, buffer);
    client.publish(TOPIC_DIST, buffer);
    Serial.printf("│ 📏 Jarak:        %6.2f cm ", distance);
    if (distance < DISTANCE_THRESHOLD && distance > 0) Serial.print("⚠️ DEKAT");
    Serial.println();
  }
  
  if (HAS_LDR) {
    sprintf(buffer, "%d", lightLevel);
    client.publish(TOPIC_LIGHT, buffer);
    Serial.printf("│ 💡 Cahaya:       %6d %% ", lightLevel);
    if (lightLevel < LIGHT_THRESHOLD) Serial.print("🌙 GELAP");
    Serial.println();
  }
  
  if (HAS_PIR) {
    client.publish(TOPIC_MOTION, motionDetected ? "1" : "0");
    Serial.printf("│ 🚶 Gerakan:      %-26s │\n", 
                  motionDetected ? "⚠️ TERDETEKSI!" : "Tidak ada");
  }
  
  Serial.println("├──────────────────────────────────────────────┤");
  Serial.printf("│ 💡 LED: %-3s | 🔔 Buzzer: %-3s | 🔌 Relay: %-3s │\n", 
                ledState ? "ON" : "OFF",
                buzzerState ? "ON" : "OFF",
                relayState ? "ON" : "OFF");
  Serial.println("└──────────────────────────────────────────────┘\n");
}

void publishActuatorStatus() {
  client.publish(TOPIC_LED_STATUS, ledState ? "ON" : "OFF");
  client.publish(TOPIC_BUZZER_STATUS, buzzerState ? "ON" : "OFF");
  client.publish(TOPIC_RELAY_STATUS, relayState ? "ON" : "OFF");
}

// ==================== MAIN LOOP ====================
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  unsigned long currentMillis = millis();
  
  // Read sensors
  if (currentMillis - lastSensorRead >= sensorInterval) {
    lastSensorRead = currentMillis;
    readAllSensors();
  }
  
  // Run automation logic
  if (currentMillis - lastAutomation >= automationInterval) {
    lastAutomation = currentMillis;
    runAutomation();  // INI YANG PENTING - Logic automation
  }
  
  // Publish data
  if (currentMillis - lastPublish >= publishInterval) {
    lastPublish = currentMillis;
    publishSensorData();
  }
}