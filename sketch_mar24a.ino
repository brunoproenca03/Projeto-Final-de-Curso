#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define LCD_SDA 21
#define LCD_SCL 47
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define PMU_SDA 15
#define PMU_SCL 7
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"
XPowersPMU PMU;

#define TINY_GSM_MODEM_SIM7080
#define DUMP_AT_COMMANDS
#define TINY_GSM_RX_BUFFER 1024

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(Serial1, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(Serial1);
#endif

const char *apn = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

const char *mqttServer = "HydrometerIoTHub.azure-devices.net";
const int mqttPort = 8883;
const char *deviceId = "HydrometerDevice-001";
const char *mqttClientId = "HydrometerDevice-001";
const char *mqttUsername = "HydrometerIoTHub.azure-devices.net/HydrometerDevice-001/?api-version=2018-06-30";
const char *mqttPassword = "SharedAccessSignature sr=HydrometerIoTHub.azure-devices.net%2Fdevices%2FHydrometerDevice-001&sig=VLes44s%2FBttc8dJnkipNVAblliZ9S1%2FG1Mb6cHgAdIE%3D&se=1755436704";

TinyGsmClientSecure gsmClient(modem);
PubSubClient mqttClient(gsmClient);

#define FLOW_SENSOR_PIN 13
volatile uint16_t pulseCount = 0;
float calibrationFactor = 360.0F;
unsigned long lastSend = 0;
const unsigned long sendInterval = 10000UL;
float totalLitros = 0.0F;

#define EEPROM_ADDR 0

float lat2 = 0.0F;
float lon2 = 0.0F;
bool gpsFixOk = false;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}
void stopGNSS();
void sendToMQTT(float liters, uint16_t pulses, const char *timestamp, int yy, int MM, int dd, int hh, int mm, int ss);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(3000);
  Serial.println("Inicializando...");

  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  delay(50);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando LCD...");

  EEPROM.begin(8);
  EEPROM.get(EEPROM_ADDR, totalLitros);

  Wire1.begin(PMU_SDA, PMU_SCL);
  if (!PMU.begin(Wire1, AXP2101_SLAVE_ADDRESS, PMU_SDA, PMU_SCL)) {
    lcd.setCursor(0, 1);
    lcd.print("Falha no PMU");
    while (1) delay(5000);
  }
  PMU.setDC3Voltage(3000);
  PMU.enableDC3();
  PMU.setBLDO2Voltage(3300);
  PMU.enableBLDO2();
  PMU.disableTSPinMeasure();

  Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);
  pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
  int retry = 0;
  while (!modem.testAT(1000)) {
    if (retry++ > 6) {
      digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
      delay(1000);
      digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
      retry = 0;
    }
  }

  if (modem.getSimStatus() != SIM_READY) {
    lcd.setCursor(0, 1);
    lcd.print("SIM ausente!     ");
    while (1) delay(5000);
  }

  modem.sendAT("+CFUN=0");
  delay(1000);
  modem.setNetworkMode(2);
  modem.setPreferredMode(1);
  modem.sendAT("+CFUN=1");
  delay(3000);
  modem.sendAT("+CASSLCFG=0,\"IGNORELOCALTIME\",1");

  while (modem.getRegistrationStatus() != REG_OK_HOME) {
    Serial.print('.');
    delay(1000);
  }
  lcd.setCursor(0, 1);
  lcd.print("Rede OK          ");

  modem.gprsConnect(apn, gprsUser, gprsPass);

  modem.disableGPS();
  delay(500);
  modem.sendAT("+CGNSMOD=1,0,1,0,0");
  modem.waitResponse();
  modem.sendAT("+SGNSCMD=2,1000,0,0");
  modem.waitResponse();
  modem.sendAT("+SGNSCMD=0");
  modem.waitResponse();
  delay(500);
  modem.enableGPS();

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setBufferSize(1024);
  mqttClient.setKeepAlive(60);
}

void loop() {
  if (millis() - lastSend < sendInterval) return;

  detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
  uint16_t pulses = pulseCount;
  pulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
  float liters = (float)pulses / calibrationFactor;

  String datetime;
  modem.sendAT("+CCLK?");
  if (modem.waitResponse(1000L, datetime) != 1) return;

  int yy, MM, dd, hh, mm, ss;
  int si = datetime.indexOf('"');
  int ei = datetime.indexOf('"', si + 1);
  if (si == -1 || ei == -1) return;
  String dtStr = datetime.substring(si + 1, ei);
  sscanf(dtStr.c_str(), "%2d/%2d/%2d,%2d:%2d:%2d", &yy, &MM, &dd, &hh, &mm, &ss);
  char timestamp[25];
  snprintf(timestamp, sizeof(timestamp), "20%02d-%02d-%02dT%02d:%02d:%02dZ", yy, MM, dd, hh, mm, ss);

  if (!gpsFixOk) {
    if (modem.getGPS(&lat2, &lon2)) {
      gpsFixOk = true;
      stopGNSS();
    }
  }

  totalLitros += liters;
  EEPROM.put(EEPROM_ADDR, totalLitros);
  EEPROM.commit();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L:");
  lcd.print(liters, 2);
  lcd.print(" P:");
  lcd.print(pulses);
  lcd.setCursor(0, 1);
  lcd.printf("%08.2f L", totalLitros);

  if (liters > 0.001) {
    sendToMQTT(liters, pulses, timestamp, yy, MM, dd, hh, mm, ss);
  }

  lastSend = millis();
}

void stopGNSS() {
  modem.sendAT("+SGNSCMD=0");
  modem.waitResponse();
  modem.disableGPS();
}

void sendToMQTT(float liters, uint16_t pulses, const char *timestamp,
                int yy, int MM, int dd, int hh, int mm, int ss) {

  // Conectar ao broker MQTT
  if (!mqttClient.connect(mqttClientId, mqttUsername, mqttPassword)) {
    Serial.print("[MQTT ERRO] Código: ");
    Serial.println(mqttClient.state());
    return;
  }
  Serial.println("[MQTT] Conectado ao broker Azure!");

  // Criar JSON com dados
  StaticJsonDocument<384> jsonDoc;
  jsonDoc["litros"] = liters;
  jsonDoc["pulsos"] = pulses;
  jsonDoc["DataHora"] = timestamp;
  jsonDoc["Ano"] = 2000 + yy;  // Porque vem em formato "25" para 2025
  jsonDoc["Mes"] = MM;
  jsonDoc["Dia"] = dd;
  jsonDoc["Hora"] = hh;
  jsonDoc["Minuto"] = mm;
  jsonDoc["Segundo"] = ss;
  jsonDoc["Lat"] = lat2;
  jsonDoc["Lon"] = lon2;
  jsonDoc["Total"] = totalLitros;

  String payload;
  serializeJson(jsonDoc, payload);
  String topic = "devices/" + String(deviceId) + "/messages/events/";

  // Publicar
  if (mqttClient.publish(topic.c_str(), payload.c_str())) {
    Serial.println("[MQTT] Publicado com sucesso:");
    Serial.println(payload);
  } else {
    Serial.println("[MQTT] Falha ao publicar mensagem.");
  }

  delay(200);  // Pequeno atraso para garantir que a mensagem foi enviada

  // Desconectar MQTT após envio
  mqttClient.disconnect();
  Serial.println("[MQTT] Desconectado após envio.");
