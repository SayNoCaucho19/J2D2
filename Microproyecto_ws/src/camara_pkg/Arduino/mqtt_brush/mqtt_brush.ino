// ===========================================================
// ===============   AQUABOT MQTT + SENSOR FZ0430  ============
// ===========================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_adc_cal.h"
#include "HX711.h"

// ====== CONFIGURACIÓN WiFi y MQTT ======
const char* ssid = "AquaBotNet";
const char* password = "aquabot123";
const char* mqtt_broker = "10.42.0.1";
const int mqtt_port = 1883;

const char* mqtt_topic_commands = "aquabot/motors";
const char* mqtt_topic_status = "aquabot/status";
const char* mqtt_topic_battery = "aquabot/battery";
const char* mqtt_topic_peso = "aquabot/peso";

// ====== PINES Y CONFIGURACIÓN PWM ======
const int escPinLeft = 19;
const int escPinRight = 21;
const int voltageSensorPin = 34;

// ====== PWM CONFIGURACIÓN - MÉTODO QUE FUNCIONA ======
const int freq = 50;
const int resolution = 16;
const int ledChannelLeft = 0;   // ← AGREGADO: Canal PWM
const int ledChannelRight = 1;  // ← AGREGADO: Canal PWM

// ====== SENSOR FZ0430 ======
const float VOLTAGE_DIVIDER = 7.6;
float CALIB_FACTOR = 0.66095;
float CALIB_OFFSET = 0.0;
const int DEFAULT_VREF = 1100;
esp_adc_cal_characteristics_t adc_chars;

// ====== CONFIGURACIÓN DE BATERÍA ======
const float BATTERY_MIN_VOLTAGE = 10.5;
const float BATTERY_MAX_VOLTAGE = 12.6;
const float BATTERY_CRITICAL = 10.8;
const float BATTERY_WARNING = 11.1;

// ====== INTERVALOS ======
const unsigned long COMMAND_TIMEOUT = 1000;
const unsigned long BATTERY_READ_INTERVAL = 1000;
const unsigned long HEARTBEAT_INTERVAL = 1000;
const unsigned long LOADCELL_INTERVAL = 500;

// ====== SENSOR HX711 ======
HX711 scale;
const int HX711_DOUT = 26;
const int HX711_SCK  = 27;
float calibration_factor = -7050.0;

// ====== VARIABLES ======
float leftMotorValue = 0.0;
float rightMotorValue = 0.0;

unsigned long lastCommandTime = 0;
unsigned long lastBatteryRead = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastLoadCellRead = 0;

float batteryVoltage = 0.0;
float batteryPercentage = 0.0;

// ====== OBJETOS MQTT ======
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ====== DECLARACIONES ======
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void readBatteryVoltage();
void applyMotorValues();
void writeESC(int channel, int us);
void emergencyStop();
void sendHeartbeat();
void sendBatteryAlert();
String getBatteryStatus();
int floatToPulse(float value);
bool testTCPConnection(const char* host, int port);
void readLoadCell();

// ===========================================================
// =========================== SETUP ==========================
// ===========================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Iniciando AquaBot ===");

  // ===== CONFIGURAR ADC =====
  analogReadResolution(12);
  analogSetPinAttenuation(voltageSensorPin, ADC_11db);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
  pinMode(voltageSensorPin, INPUT);

  // ===== PWM - MÉTODO CORRECTO =====
  Serial.println("Configurando PWM...");
  ledcSetup(ledChannelLeft, freq, resolution);   // ← CORREGIDO
  ledcSetup(ledChannelRight, freq, resolution);  // ← CORREGIDO
  ledcAttachPin(escPinLeft, ledChannelLeft);     // ← CORREGIDO
  ledcAttachPin(escPinRight, ledChannelRight);   // ← CORREGIDO

  Serial.println("Armando ESCs...");
  writeESC(ledChannelLeft, 1500);   // ← CORREGIDO: usa canal
  writeESC(ledChannelRight, 1500);  // ← CORREGIDO: usa canal
  delay(3000);
  Serial.println("ESCs listos");

  // ===== SENSOR HX711 =====
  Serial.println("Iniciando HX711...");
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(calibration_factor);
  scale.tare();
  Serial.println("HX711 listo");

  // ===== CONEXIÓN WiFi =====
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✓ WiFi conectado");
  Serial.println(WiFi.localIP());

  // ===== TEST TCP =====
  testTCPConnection(mqtt_broker, mqtt_port);

  // ===== CONFIGURACIÓN MQTT =====
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  reconnectMQTT();

  Serial.println("\n=== Sistema listo ===\n");
}

// ===========================================================
// ============================ LOOP ==========================
// ===========================================================
void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Lectura batería
  if (millis() - lastBatteryRead > BATTERY_READ_INTERVAL) {
    readBatteryVoltage();
    lastBatteryRead = millis();
  }

  // Lectura sensor de peso
  if (millis() - lastLoadCellRead > LOADCELL_INTERVAL) {
    readLoadCell();
    lastLoadCellRead = millis();
  }

  // Timeout motores
  if (millis() - lastCommandTime > COMMAND_TIMEOUT && lastCommandTime > 0) {
    leftMotorValue = 0.0;
    rightMotorValue = 0.0;
  }

  applyMotorValues();

  // Heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }

  // Alerta batería
  if (batteryVoltage < BATTERY_CRITICAL && batteryVoltage > 5.0) {
    sendBatteryAlert();
  }

  delay(20);
}

// ===========================================================
// =============== LECTURA SENSOR HX711 =======================
// ===========================================================
void readLoadCell() {
  if (!scale.is_ready()) {
    Serial.println("HX711 no listo");
    return;
  }

  float weight = scale.get_units(5);
  float m = 17.226;
  float b = -84.91;
  weight = weight * m + b;

  Serial.printf("Peso: %.2f g\n", weight);

  if (mqttClient.connected()) {
    StaticJsonDocument<128> doc;
    doc["type"] = "peso";
    doc["valor_g"] = weight;
    doc["timestamp"] = millis();

    char buffer[128];
    serializeJson(doc, buffer);
    mqttClient.publish(mqtt_topic_peso, buffer);
  }
}

// ===========================================================
// ================= FUNCIONES DE ESTADO ======================
// ===========================================================
void readBatteryVoltage() {
  uint32_t adc_raw = 0;

  for (int i = 0; i < 10; i++) {
    adc_raw += analogRead(voltageSensorPin);
    delay(5);
  }
  adc_raw /= 10;

  uint32_t voltage_mV = esp_adc_cal_raw_to_voltage(adc_raw, &adc_chars);
  float v_adc = voltage_mV / 1000.0;
  float v_real = v_adc * VOLTAGE_DIVIDER;
  batteryVoltage = (v_real * CALIB_FACTOR) + CALIB_OFFSET;

  batteryPercentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                       (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  batteryPercentage = constrain(batteryPercentage, 0.0, 100.0);

  Serial.printf("ADC=%u | Pin=%.3fV | Batería=%.2fV (%.1f%%) [%s]\n",
                adc_raw, v_adc, batteryVoltage, batteryPercentage, 
                getBatteryStatus().c_str());
}

String getBatteryStatus() {
  if (batteryVoltage < BATTERY_CRITICAL) return "CRITICO";
  else if (batteryVoltage < BATTERY_WARNING) return "BAJO";
  else if (batteryPercentage < 50) return "MEDIO";
  else if (batteryPercentage < 80) return "BUENO";
  else return "EXCELENTE";
}

// ===========================================================
// ========================= MQTT ============================
// ===========================================================
void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  static int failCount = 0;
  
  unsigned long backoff = 5000 * (1 << min(failCount, 3));
  if (millis() - lastAttempt < backoff) return;
  lastAttempt = millis();

  if (!mqttClient.connected()) {
    Serial.print("Intentando conexión MQTT... ");
    
    uint8_t mac[6];
    WiFi.macAddress(mac);
    String clientId = "aquabot-";
    for (int i = 0; i < 6; i++) {
      clientId += String(mac[i], HEX);
    }
    
    Serial.print("ClientID: ");
    Serial.println(clientId);
    
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("✓ MQTT conectado!");
      failCount = 0;
      
      if (mqttClient.subscribe(mqtt_topic_commands)) {
        Serial.println("✓ Suscrito a comandos");
      } else {
        Serial.println("✗ Error al suscribirse");
      }
      
      sendHeartbeat();
    } else {
      failCount++;
      Serial.print("✗ Falló, rc=");
      Serial.print(mqttClient.state());
      Serial.print(" intentos: ");
      Serial.println(failCount);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("Error parseando JSON: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("emergency")) {
    Serial.println("¡EMERGENCIA!");
    emergencyStop();
    return;
  }

  if (doc.containsKey("left") && doc.containsKey("right")) {
    leftMotorValue = constrain((float)doc["left"], -1.0, 1.0);
    rightMotorValue = constrain((float)doc["right"], -1.0, 1.0);
    lastCommandTime = millis();
    
    Serial.printf("Motor L=%.2f R=%.2f\n", leftMotorValue, rightMotorValue);
  }
}

// ===========================================================
// =============== CONTROL DE MOTORES =========================
// ===========================================================
inline int floatToPulse(float value) {
  return constrain(1500 + (int)(value * 500.0), 1000, 2000);
}

void applyMotorValues() {
  writeESC(ledChannelLeft, floatToPulse(leftMotorValue));   // ← CORREGIDO: usa canal
  writeESC(ledChannelRight, floatToPulse(-rightMotorValue)); // ← CORREGIDO: usa canal
}

// ← FUNCIÓN CORREGIDA: Ahora convierte microsegundos a duty cycle
void writeESC(int channel, int us) {
  ledcWrite(channel, map(us, 1000, 2000, 3276, 6553));
}

// ===========================================================
// ==================== EMERGENCIA ===========================
// ===========================================================
void emergencyStop() {
  Serial.println("EJECUTANDO PARADA DE EMERGENCIA");
  
  writeESC(ledChannelLeft, 1500);   // ← CORREGIDO: usa canal
  writeESC(ledChannelRight, 1500);  // ← CORREGIDO: usa canal
  leftMotorValue = 0.0;
  rightMotorValue = 0.0;

  if (mqttClient.connected()) {
    StaticJsonDocument<128> doc;
    doc["type"] = "emergency_stop";
    doc["timestamp"] = millis() / 1000;

    char buffer[128];
    serializeJson(doc, buffer);
    mqttClient.publish(mqtt_topic_status, buffer, true);
    
    Serial.println("Emergencia publicada a MQTT");
  }
}

// ===========================================================
// ================== HEARTBEAT Y ALERTAS ====================
// ===========================================================
void sendHeartbeat() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<300> doc;
  doc["type"] = "heartbeat";
  doc["uptime"] = millis() / 1000;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["battery_voltage"] = batteryVoltage;
  doc["battery_percentage"] = batteryPercentage;
  doc["battery_status"] = getBatteryStatus();
  doc["left_motor"] = leftMotorValue;
  doc["right_motor"] = rightMotorValue;
  doc["timestamp"] = millis();

  char buffer[300];
  int len = serializeJson(doc, buffer);
  
  bool status_ok = mqttClient.publish(mqtt_topic_status, buffer);
  bool battery_ok = mqttClient.publish(mqtt_topic_battery, buffer);
  
  if (status_ok && battery_ok) {
    Serial.printf("→ Heartbeat enviado (%d bytes)\n", len);
  } else {
    Serial.println("✗ Error enviando heartbeat");
  }
}

void sendBatteryAlert() {
  static unsigned long lastAlert = 0;
  if (millis() - lastAlert < 10000) return;
  lastAlert = millis();

  if (!mqttClient.connected()) return;

  Serial.println("⚠ ALERTA DE BATERÍA BAJA");

  StaticJsonDocument<200> doc;
  doc["type"] = "battery_alert";
  doc["voltage"] = batteryVoltage;
  doc["percentage"] = batteryPercentage;
  doc["status"] = getBatteryStatus();
  doc["message"] = "BATERIA BAJA - Regresar a base";
  doc["timestamp"] = millis() / 1000;

  char buffer[200];
  serializeJson(doc, buffer);
  mqttClient.publish(mqtt_topic_battery, buffer, true);
  
  Serial.println("Alerta publicada");
}

// ===========================================================
// ================= TEST TCP CONNECTION =====================
// ===========================================================
bool testTCPConnection(const char* host, int port) {
  Serial.print("Probando conexión TCP a ");
  Serial.print(host);
  Serial.print(":");
  Serial.print(port);
  Serial.print("... ");
  
  WiFiClient testClient;
  if (testClient.connect(host, port, 10000)) {
    Serial.println("✓ TCP exitoso");
    testClient.stop();
    return true;
  } else {
    Serial.println("✗ TCP falló");
    return false;
  }
}