/*
 * Control de motores brushless ESP32 vía MQTT - OPTIMIZADO PARA BAJA LATENCIA
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ========== CONFIGURACIÓN ==========
const char* ssid = "Redmi Note 11";
const char* password = "123456789";
const char* mqtt_broker = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_topic_commands = "aquabot/motors";
const char* mqtt_topic_status = "aquabot/status";

// ========== HARDWARE ==========
const int escPinLeft = 18;
const int escPinRight = 19;
const int freq = 50;
const int ledChannelLeft = 0;
const int ledChannelRight = 1;
const int resolution = 16;

// ========== VARIABLES ==========
float leftMotorValue = 0.0;
float rightMotorValue = 0.0;
unsigned long lastCommandTime = 0;
unsigned long lastHeartbeat = 0;
const unsigned long COMMAND_TIMEOUT = 1000;
const unsigned long HEARTBEAT_INTERVAL = 5000;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup() {
  Serial.begin(115200);
  
  // Configurar PWM
  ledcSetup(ledChannelLeft, freq, resolution);
  ledcSetup(ledChannelRight, freq, resolution);
  ledcAttachPin(escPinLeft, ledChannelLeft);
  ledcAttachPin(escPinRight, ledChannelRight);
  
  // Armar ESCs
  writeESC(ledChannelLeft, 1500);
  writeESC(ledChannelRight, 1500);
  delay(3000);
  
  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  // MQTT
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  
  Serial.println("Ready");
}

void loop() {
  // Mantener conexiones
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
  }
  
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  
  // Timeout de seguridad
  if (millis() - lastCommandTime > COMMAND_TIMEOUT && lastCommandTime > 0) {
    if (leftMotorValue != 0.0 || rightMotorValue != 0.0) {
      leftMotorValue = 0.0;
      rightMotorValue = 0.0;
    }
  }
  
  // Aplicar valores
  applyMotorValues();
  
  // Heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  delay(20);
}

void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt < 5000) return;
  lastAttempt = millis();
  
  if (mqttClient.connect("esp32_aquabot")) {
    mqttClient.subscribe(mqtt_topic_commands);
    sendHeartbeat();
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) return;
  
  // Parada de emergencia
  if (doc.containsKey("emergency") && doc["emergency"] == true) {
    emergencyStop();
    return;
  }
  
  // Leer motores
  if (doc.containsKey("left") && doc.containsKey("right")) {
    leftMotorValue = constrain((float)doc["left"], -1.0, 1.0);
    rightMotorValue = constrain((float)doc["right"], -1.0, 1.0);
    lastCommandTime = millis();
  }
}

inline int floatToPulse(float value) {
  return constrain(1500 + (int)(value * 500.0), 1000, 2000);
}

void applyMotorValues() {
  writeESC(ledChannelLeft, floatToPulse(leftMotorValue));
  writeESC(ledChannelRight, floatToPulse(rightMotorValue));
}

void writeESC(int channel, int us) {
  ledcWrite(channel, map(us, 1000, 2000, 3276, 6553));
}

void emergencyStop() {
  writeESC(ledChannelLeft, 1500);
  writeESC(ledChannelRight, 1500);
  leftMotorValue = 0.0;
  rightMotorValue = 0.0;
}

void sendHeartbeat() {
  if (!mqttClient.connected()) return;
  
  StaticJsonDocument<128> doc;
  doc["type"] = "heartbeat";
  doc["uptime"] = millis() / 1000;
  doc["wifi_rssi"] = WiFi.RSSI();
  
  char buffer[128];
  serializeJson(doc, buffer);
  mqttClient.publish(mqtt_topic_status, buffer);
}