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
#define AUTO_LIGHTING true       
#define AUTO_SECURITY true       
#define AUTO_COOLING true        
#define AUTO_PROXIMITY true      

#define LIGHT_THRESHOLD 30       
#define TEMP_THRESHOLD 30.0      
#define DISTANCE_THRESHOLD 20.0  

#define PIR_DEBOUNCE_TIME 3000   
unsigned long lastMotionTime = 0;
bool motionCooldown = false;

#define LDR_INVERT true

// ==================== CONTROL MODE ====================
// NEW: Per-actuator manual override flags
bool ledManualOverride = false;
bool buzzerManualOverride = false;
bool relayManualOverride = false;

// Manual command values
bool ledManualState = false;
bool buzzerManualState = false;
bool relayManualState = false;

// Timing untuk auto-release manual override
unsigned long ledManualTime = 0;
unsigned long buzzerManualTime = 0;
unsigned long relayManualTime = 0;
#define MANUAL_TIMEOUT 30000  // 30 detik manual, kemudian kembali auto

// ==================== OBJECTS ====================
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// ==================== VARIABLES ====================
float temperature = 0, humidity = 0, distance = 0;
int lightLevel = 0;
int lightLevelRaw = 0;
bool motionDetected = false;
bool motionPrevious = false;

bool ledState = false, buzzerState = false, relayState = false;
bool ledAuto = false, buzzerAuto = false, relayAuto = false;

unsigned long lastSensorRead = 0;
unsigned long lastPublish = 0;
unsigned long lastAutomation = 0;
const long sensorInterval = 500;   
const long publishInterval = 2000;
const long automationInterval = 1000;

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

const char* TOPIC_LED_MODE = "Klp2/esp32/led/mode";      // NEW: mode status
const char* TOPIC_BUZZER_MODE = "Klp2/esp32/buzzer/mode";
const char* TOPIC_RELAY_MODE = "Klp2/esp32/relay/mode";

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  printBanner();
  initializeSensors();
  initializeActuators();
  calibrateLDR();
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║    🔀 HYBRID CONTROL MODE ACTIVE! 🔀              ║");
  Serial.println("║                                                    ║");
  Serial.println("║  ✅ Automation: Aktuator otomatis dari sensor     ║");
  Serial.println("║  ✅ Manual Control: Override kapan saja!          ║");
  Serial.println("║  ✅ Auto-release: Kembali auto setelah 30s        ║");
  Serial.println("║                                                    ║");
  Serial.println("║  Automation Scenarios:                            ║");
  Serial.printf("║  🌙 Cahaya < %d%% → LED ON                         ║\n", LIGHT_THRESHOLD);
  Serial.println("║  🚶 Motion Detected → Alarm (3s cooldown)        ║");
  Serial.printf("║  🌡️  Suhu > %.0f°C → Relay ON                      ║\n", TEMP_THRESHOLD);
  Serial.printf("║  📏 Jarak < %.0fcm → Buzzer ON                     ║\n", DISTANCE_THRESHOLD);
  Serial.println("╚════════════════════════════════════════════════════╝\n");
}

void printBanner() {
  Serial.println("\n╔═════════════════════════════════════════════════╗");
  Serial.println("║   ESP32 IoT - HYBRID CONTROL SYSTEM            ║");
  Serial.println("║   Automation + Manual Control Coexist          ║");
  Serial.println("╚═════════════════════════════════════════════════╝\n");
}

void calibrateLDR() {
  Serial.println("🔧 Calibrating LDR Sensor...");
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    int raw = analogRead(LDR_PIN);
    sum += raw;
    delay(100);
  }
  int avgRaw = sum / 10;
  Serial.printf("   LDR Average Raw Value: %d (0-4095)\n", avgRaw);
  if (avgRaw < 500) {
    Serial.println("   ⚠️  WARNING: LDR reads very LOW");
  } else if (avgRaw > 3500) {
    Serial.println("   ⚠️  WARNING: LDR reads very HIGH");
  } else {
    Serial.println("   ✓ LDR reading normal");
  }
  Serial.println();
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
    Serial.println("  ✓ PIR Sensor (3s debounce)");
  }
}

void initializeActuators() {
  Serial.println("\n🔧 Initializing Actuators...");
  if (HAS_LED) {
    pinMode(LED_RED_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, LOW);
    Serial.println("  ✓ LED (Hybrid Control)");
  }
  if (HAS_BUZZER) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("  ✓ Buzzer (Hybrid Control)");
  }
  if (HAS_RELAY) {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("  ✓ Relay (Hybrid Control)");
  }
}

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

void reconnect() {
  while (!client.connected()) {
    Serial.print("🔄 MQTT...");
    
    String fullClientId = clientId + String(random(0xffff), HEX);
    
    if (client.connect(fullClientId.c_str())) {
      Serial.println(" ✓");
      
      client.subscribe(TOPIC_LED);
      client.subscribe(TOPIC_BUZZER);
      client.subscribe(TOPIC_RELAY);
      
      publishActuatorStatus();
      
    } else {
      Serial.print(" ✗ rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// ==================== MQTT CALLBACK (HYBRID CONTROL) ====================
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("\n📩 [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);
  
  // LED Manual Control
  if (String(topic) == TOPIC_LED && HAS_LED) {
    ledManualOverride = true;
    ledManualTime = millis();
    ledManualState = (message == "ON" || message == "1");
    
    digitalWrite(LED_RED_PIN, ledManualState ? HIGH : LOW);
    ledState = ledManualState;
    
    Serial.printf("  💡 LED: MANUAL %s (override automation)\n", 
                  ledManualState ? "ON" : "OFF");
    Serial.println("     Auto-release in 30 seconds...");
  }
  
  // Buzzer Manual Control
  if (String(topic) == TOPIC_BUZZER && HAS_BUZZER) {
    buzzerManualOverride = true;
    buzzerManualTime = millis();
    buzzerManualState = (message == "ON" || message == "1");
    
    digitalWrite(BUZZER_PIN, buzzerManualState ? HIGH : LOW);
    buzzerState = buzzerManualState;
    
    Serial.printf("  🔔 Buzzer: MANUAL %s (override automation)\n", 
                  buzzerManualState ? "ON" : "OFF");
    Serial.println("     Auto-release in 30 seconds...");
  }
  
  // Relay Manual Control
  if (String(topic) == TOPIC_RELAY && HAS_RELAY) {
    relayManualOverride = true;
    relayManualTime = millis();
    relayManualState = (message == "ON" || message == "1");
    
    digitalWrite(RELAY_PIN, relayManualState ? HIGH : LOW);
    relayState = relayManualState;
    
    Serial.printf("  🔌 Relay: MANUAL %s (override automation)\n", 
                  relayManualState ? "ON" : "OFF");
    Serial.println("     Auto-release in 30 seconds...");
  }
  
  publishActuatorStatus();
}

// ==================== CHECK MANUAL TIMEOUT ====================
void checkManualTimeout() {
  unsigned long currentTime = millis();
  
  // LED timeout check
  if (ledManualOverride && (currentTime - ledManualTime > MANUAL_TIMEOUT)) {
    ledManualOverride = false;
    Serial.println("\n⏱️  LED: Manual timeout → Back to AUTOMATION");
    publishActuatorStatus();
  }
  
  // Buzzer timeout check
  if (buzzerManualOverride && (currentTime - buzzerManualTime > MANUAL_TIMEOUT)) {
    buzzerManualOverride = false;
    Serial.println("\n⏱️  Buzzer: Manual timeout → Back to AUTOMATION");
    publishActuatorStatus();
  }
  
  // Relay timeout check
  if (relayManualOverride && (currentTime - relayManualTime > MANUAL_TIMEOUT)) {
    relayManualOverride = false;
    Serial.println("\n⏱️  Relay: Manual timeout → Back to AUTOMATION");
    publishActuatorStatus();
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
  if (duration == 0) return 999;
  
  return duration * 0.034 / 2;
}

int readLightLevel() {
  if (!HAS_LDR) return 50;
  
  lightLevelRaw = analogRead(LDR_PIN);
  
  if (LDR_INVERT) {
    return map(lightLevelRaw, 0, 4095, 100, 0);
  } else {
    return map(lightLevelRaw, 0, 4095, 0, 100);
  }
}

bool readMotion() {
  if (!HAS_PIR) return false;
  
  bool currentState = digitalRead(PIR_PIN) == HIGH;
  unsigned long currentTime = millis();
  
  if (currentState && !motionPrevious) {
    if (!motionCooldown) {
      motionPrevious = true;
      lastMotionTime = currentTime;
      motionCooldown = true;
      return true;
    }
  } else if (!currentState && motionPrevious) {
    motionPrevious = false;
  }
  
  if (motionCooldown && (currentTime - lastMotionTime > PIR_DEBOUNCE_TIME)) {
    motionCooldown = false;
  }
  
  return motionPrevious;
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
  }
  
  if (HAS_LDR) {
    lightLevel = readLightLevel();
  }
  
  if (HAS_PIR) {
    motionDetected = readMotion();
  }
}

// ==================== HYBRID AUTOMATION LOGIC ====================
void runAutomation() {
  bool ledPrev = ledAuto;
  bool buzzerPrev = buzzerAuto;
  bool relayPrev = relayAuto;
  
  // Reset automation states
  ledAuto = false;
  buzzerAuto = false;
  relayAuto = false;
  
  // SCENARIO 1: Smart Lighting
  if (AUTO_LIGHTING && HAS_LDR) {
    if (lightLevel < LIGHT_THRESHOLD) {
      ledAuto = true;
    }
  }
  
  // SCENARIO 2: Security Alarm
  if (AUTO_SECURITY && HAS_PIR) {
    if (motionDetected) {
      buzzerAuto = true;
      ledAuto = true;
    }
  }
  
  // SCENARIO 3: Auto Cooling
  if (AUTO_COOLING && HAS_DHT) {
    if (temperature > TEMP_THRESHOLD) {
      relayAuto = true;
    }
  }
  
  // SCENARIO 4: Proximity Alert
  if (AUTO_PROXIMITY && HAS_ULTRASONIC) {
    if (distance > 0 && distance < DISTANCE_THRESHOLD) {
      if (!buzzerAuto) {
        buzzerAuto = true;
      }
    }
  }
  
  // ==================== HYBRID CONTROL LOGIC ====================
  // Apply automation ONLY if NOT manually overridden
  
  // LED: Manual override OR automation
  if (!ledManualOverride) {
    ledState = ledAuto;
    digitalWrite(LED_RED_PIN, ledState ? HIGH : LOW);
  }
  // else: keep manual state (already set in callback)
  
  // Buzzer: Manual override OR automation
  if (!buzzerManualOverride) {
    buzzerState = buzzerAuto;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
  }
  
  // Relay: Manual override OR automation
  if (!relayManualOverride) {
    relayState = relayAuto;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
  }
  
  // Log changes (only for automation, not manual)
  if (!ledManualOverride && ledState != ledPrev) {
    Serial.println("\n🤖 ═══ AUTOMATION TRIGGERED ═══");
    Serial.printf("   💡 LED: %s ", ledState ? "ON" : "OFF");
    if (motionDetected) {
      Serial.println("(Motion Security)");
    } else if (lightLevel < LIGHT_THRESHOLD) {
      Serial.println("(Cahaya rendah)");
    } else {
      Serial.println();
    }
    publishActuatorStatus();
  }
  
  if (!buzzerManualOverride && buzzerState != buzzerPrev) {
    Serial.println("\n🤖 ═══ AUTOMATION TRIGGERED ═══");
    Serial.printf("   🔔 Buzzer: %s ", buzzerState ? "ON" : "OFF");
    if (motionDetected) {
      Serial.println("(Security Alert!)");
    } else if (distance < DISTANCE_THRESHOLD) {
      Serial.println("(Proximity Alert!)");
    } else {
      Serial.println();
    }
    publishActuatorStatus();
  }
  
  if (!relayManualOverride && relayState != relayPrev) {
    Serial.println("\n🤖 ═══ AUTOMATION TRIGGERED ═══");
    Serial.printf("   🔌 Relay: %s ", relayState ? "ON" : "OFF");
    if (temperature > TEMP_THRESHOLD) {
      Serial.printf("(Suhu: %.1f°C)\n", temperature);
    } else {
      Serial.println();
    }
    publishActuatorStatus();
  }
}

// ==================== PUBLISH ====================
void publishSensorData() {
  char buffer[8];
  
  Serial.println("┌──────────────────────────────────────────────────┐");
  Serial.println("│ 📊 SENSOR DATA                                   │");
  Serial.println("├──────────────────────────────────────────────────┤");
  
  if (HAS_DHT) {
    dtostrf(temperature, 1, 2, buffer);
    client.publish(TOPIC_TEMP, buffer);
    Serial.printf("│ 🌡️  Suhu:         %6.2f °C ", temperature);
    if (temperature > TEMP_THRESHOLD) Serial.print("⚠️ PANAS");
    Serial.println();
    
    dtostrf(humidity, 1, 2, buffer);
    client.publish(TOPIC_HUMID, buffer);
    Serial.printf("│ 💧 Kelembaban:   %6.2f %%                      │\n", humidity);
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
    Serial.printf("│ 💡 Cahaya:       %3d%% (Raw:%4d) ", lightLevel, lightLevelRaw);
    if (lightLevel < LIGHT_THRESHOLD) Serial.print("🌙 GELAP");
    Serial.println();
  }
  
  if (HAS_PIR) {
    client.publish(TOPIC_MOTION, motionDetected ? "1" : "0");
    Serial.printf("│ 🚶 Gerakan:      %-28s │\n", 
                  motionDetected ? "⚠️ TERDETEKSI!" : "Tidak ada");
  }
  
  Serial.println("├──────────────────────────────────────────────────┤");
  Serial.printf("│ 💡 LED: %-3s [%s] ", 
                ledState ? "ON" : "OFF",
                ledManualOverride ? "MANUAL" : "AUTO  ");
  Serial.printf("🔔 Buzzer: %-3s [%s] ", 
                buzzerState ? "ON" : "OFF",
                buzzerManualOverride ? "MANUAL" : "AUTO  ");
  Serial.printf("│\n│ 🔌 Relay: %-3s [%s]%*s│\n", 
                relayState ? "ON" : "OFF",
                relayManualOverride ? "MANUAL" : "AUTO  ",
                29, "");
  Serial.println("└──────────────────────────────────────────────────┘\n");
}

void publishActuatorStatus() {
  // Publish actuator states
  client.publish(TOPIC_LED_STATUS, ledState ? "ON" : "OFF");
  client.publish(TOPIC_BUZZER_STATUS, buzzerState ? "ON" : "OFF");
  client.publish(TOPIC_RELAY_STATUS, relayState ? "ON" : "OFF");
  
  // Publish control modes
  client.publish(TOPIC_LED_MODE, ledManualOverride ? "MANUAL" : "AUTO");
  client.publish(TOPIC_BUZZER_MODE, buzzerManualOverride ? "MANUAL" : "AUTO");
  client.publish(TOPIC_RELAY_MODE, relayManualOverride ? "MANUAL" : "AUTO");
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
  
  if (currentMillis - lastAutomation >= automationInterval) {
    lastAutomation = currentMillis;
    checkManualTimeout();  // Check if manual override expired
    runAutomation();        // Run automation for non-overridden actuators
  }
  
  if (currentMillis - lastPublish >= publishInterval) {
    lastPublish = currentMillis;
    publishSensorData();
  }
}