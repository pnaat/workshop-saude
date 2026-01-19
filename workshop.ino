
// Workshop PNAAT – Monitor de Frequência Cardíaca Conectado
// Dispositivo: Arduino Nano RP2040 Connect
// Sensor: MAX30102 (SparkFun MAX3010x / MAX30105 library)
//
// Bibliotecas necessárias:
//  - WiFiNINA (Arduino)
//  - PubSubClient (knolleary)
//  - SparkFun MAX3010x Sensor Library (inclui MAX30105.h e heartRate.h)

#include <Arduino.h>
#include <Wire.h>
#include <WiFiNINA.h>        // Wi-Fi para Nano RP2040 Connect (NINA-W102)
#include <PubSubClient.h>    // MQTT
#include "MAX30105.h"        // Biblioteca SparkFun MAX30105 (MAX30102 compatível)
#include "heartRate.h"       // Algoritmo de detecção de batimento da SparkFun

// ========================= CONFIGURAÇÕES =========================

const char* WIFI_SSID     = "Projeto-Softex";     // SSID do AP  - ALTERAR
const char* WIFI_PASSWORD = "Softex@1234";        // Senha do AP - ALTERAR

const char* MQTT_HOST     = "d21541ee50b24da2bc981ecbbce1478b.s1.eu.hivemq.cloud";     // IP do broker HiveMQ
const uint16_t MQTT_PORT  = 8883;               // Porta padrão sem TLS
const char* MQTT_USERNAME = "workshop";         // Usuário do broker 
const char* MQTT_PASSWORD = "Workshop1";        // Senha do broker 

const char* DEVICE_ID     = "nano-001";         // ID único do kit - ALTERAR

// Tópicos MQTT
String baseTopic   = String("workshop");
String statusTopic = baseTopic + "/status";  // Retained: online/offline
String hrTopic     = baseTopic + "/hr";      // JSON com bpm + qualidade
String eventTopic  = baseTopic + "/event";   // eventos (finger_on/off)

// Publicação
const uint32_t PUBLISH_INTERVAL_MS = 1000;    // 1 segundo (dashboard)

// ========================= OBJETOS GLOBAIS =========================
WiFiSSLClient wifiClient;
PubSubClient mqttClient(wifiClient);
MAX30105 sensor;

bool fingerOn = false;

const byte RATE_SIZE = 4; 
const byte SAMP_SIZE = 127;
byte rates[SAMP_SIZE]; 
byte rateSpot = 0;

float beatsPerMinute;
float beatAvg = 0;
long delta;

// Controle de tempo
uint32_t tStartMs = 0;
uint32_t lastPublishMs = 0;

// ========================= FUNÇÕES AUXILIARES =========================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Conectando ao Wi-Fi '" ); Serial.print(WIFI_SSID); Serial.println("'...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - t0 > 15000) {
      Serial.println("Timeout Wi-Fi. Reiniciando tentativa...");
      t0 = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
  Serial.println("Wi-Fi conectado!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void connectMQTT() {
  if (mqttClient.connected()) return;
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  String clientId = String(DEVICE_ID) + "-" + String(random(0xFFFF), HEX);

  // Last Will and Testament (LWT)
  const char* willTopic   = statusTopic.c_str();
  const char* willPayload = "offline";
  bool willRetain = true;
  uint8_t willQos = 1;

  Serial.print("Conectando ao MQTT em "); Serial.print(MQTT_HOST); Serial.print(":"); Serial.println(MQTT_PORT);

  if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD,
                         willTopic, willQos, willRetain, willPayload, true)) {
    Serial.println("MQTT conectado.");
  } else {
    Serial.print("Falha MQTT, rc="); Serial.print(mqttClient.state()); Serial.println(". Nova tentativa em breve.");
  }
}

void ensureConnections() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqttClient.connected()) connectMQTT();
}

// Publica evento simples
void publishEvent(const char* evt) {
  if (!mqttClient.connected()) return;
  mqttClient.publish(eventTopic.c_str(), evt);
}

// Construir e publicar JSON manualmente
void publishHR() {
  if (!mqttClient.connected()) return;
  uint32_t tRelMs = millis() - tStartMs;
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"%s/%.f\"}",
           DEVICE_ID, beatAvg);
  mqttClient.publish(hrTopic.c_str(), payload);
}

// ========================= SETUP DO SENSOR =========================
bool setupMAX30102() {
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 não encontrado. Verifique conexões/SDA/SCL.");
    return false;
  }

  sensor.setup();
  sensor.setPulseAmplitudeRed(0x0A); // intensidade baixa no vermelho para economia
  sensor.setPulseAmplitudeIR(0x1F);  // IR principal
  Serial.println("MAX30102 configurado");
  return true;
}


// ========================= SETUP/LOOP =========================
void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== PNAAT Workshop ===");

  if (!setupMAX30102()) {
    //
  }

  connectWiFi();
  mqttClient.setKeepAlive(30);
  connectMQTT();

  tStartMs = millis();
  lastPublishMs = millis();
}

void loop() {
  ensureConnections();
  mqttClient.loop(); // mantém sessão MQTT

  // Leitura do MAX30102
  uint32_t ir = sensor.getIR();

  // Detecção de batimento
    if (checkForBeat(ir) == true) {
      // Intervalo entre batimentos em milissegundos
      static uint32_t lastBeat = 0;
      uint32_t now = millis();
      uint32_t delta = now - lastBeat;
      lastBeat = now;                           
      beatsPerMinute = 60 / (delta / 1000.0);   

      if (beatsPerMinute < 255 && beatsPerMinute > 20 && delta < 1000 && delta > 0){
        int measure = (byte) beatsPerMinute;
        rates[rateSpot++] = measure;
        rateSpot %= SAMP_SIZE; 
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++){
          beatAvg += rates[(rateSpot - x - 1) % SAMP_SIZE];
        }
        beatAvg /= RATE_SIZE;
        beatAvg = 0.8f * beatAvg + 0.2f * beatsPerMinute;
      }
    }

  sensor.nextSample(); // avança FIFO
  

  // Publicação periódica
  uint32_t nowMs = millis();
  if (ir < 65000){
      Serial.print(" Aproxime o dedo");
    } 
    else if (beatAvg < 50) {
      Serial.print(" Leitura ="); 
      Serial.print(beatsPerMinute);
      Serial.print(", Medindo"); 
    }
    else{
      Serial.print(" Leitura ="); 
      Serial.print(beatsPerMinute);
      Serial.print(", BPM =");
      Serial.print(beatAvg);
      if (nowMs - lastPublishMs >= PUBLISH_INTERVAL_MS) {
        Serial.print("publish");
        publishHR();
        lastPublishMs = nowMs;
      }
      
    }
  
  Serial.println();
  

  // Pequena pausa para estabilidade
  delay(2);
}
