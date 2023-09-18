// 1. Bibliotheken importieren
#include <WiFi.h>
#include <PubSubClient.h>
#include <ShiftRegister74HC595.h>
#include <Adafruit_GPS.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MHZ19.h"


MHZ19 myMHZ19;
HardwareSerial mySerial(2);

// 2. Konstanten definieren
const int SENSOR_PIN = 14, MQTT_PORT = 1024, OTA_PORT = 80, SERIAL_BAUD_RATE = 115200, GPS_BAUD_RATE = 9600;
const int SER_Pin = 7, RCLK_Pin = 5, SRCLK_Pin = 6, numOfShiftRegisters = 1, ssrPin = 13;
const int PWM_FREQUENCY = 1000, PWM_RESOLUTION = 8;
const float MIN_PRESSURE = 0.0, MAX_PRESSURE = 100.0;
const double outlierThreshold = 10.0;
const float alpha = 0.2;
const int DS18B20_PIN = 9;
const unsigned long HEARTBEAT_INTERVAL = 2000;
const unsigned long HEAP_SEND_INTERVAL = 10000;
const unsigned long CHECK_INTERVAL = 10000;

// 3. Globale Variablen definieren
WebServer server(80);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
Adafruit_GPS GPS(&Serial1);
ShiftRegister74HC595<numOfShiftRegisters> sr(SER_Pin, RCLK_Pin, SRCLK_Pin);
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long heatingStartTime, lastExecutionTime, lastGPSCheckTime, lastGPSSyncTime, currentMillis, lastExecutionTime1, currentTemperature, lastSendTime, lastExecutionTimeDruck, lastPressureSentTime, lastHeartbeatTime, lastCO2SendTime, lastCheckTime, lastHeapSendTime, lastTempSentTime, startTime;
double filteredLatitude, filteredLongitude, previousLatitude, previousLongitude, kalmanGain;
float currentSpeedKnots, currentSpeedkmh, pressure, MinDruck, pressure_cm, pressureCmFiltered = 0, lastSentPressureCm = 0.0, voltage, pressure_kPa, pressure_L;
bool relayStates[6] = { false, false, false, false, false, false }, Wasserversorgung = false, Boilerheizung = false, UVCLicht = false, isOutlier = false, isParked = false, Kuehlschrankgross, Kuehlschrankklein, Inverter, ssrState = false, waterPressureAlarm = false;
int disconnectCount = 0, ssrPower = 100, sensorValue, pwmValue, co2_ppm;
char tempMsg[50], heapMsg[50], latMsg[50], lonMsg[50], speedMsg[50], altMsg[50], c, pressureMsg[50] = { 0 }, co2Msg[50], relayTopic[50];
const char* topicStr;
String lastDisconnectReason = "Unknown";

// 4. Netzwerkkonfiguration
char ssid[] = "ssid";
char password[] = "password";
const char* mqtt_server = "192.168.0.1";


// 5. Funktionen und Hilfsfunktionen
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const char* resetReason(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN: return "UNKNOWN_RESET";
    case ESP_RST_POWERON: return "POWERON_RESET";
    case ESP_RST_EXT: return "EXTERNAL_RESET";
    case ESP_RST_SW: return "SOFTWARE_RESET";
    case ESP_RST_PANIC: return "SOFTWARE_PANIC_RESET";
    case ESP_RST_INT_WDT: return "WATCHDOG_RESET";
    case ESP_RST_TASK_WDT: return "TASK_WATCHDOG_RESET";
    case ESP_RST_WDT: return "OTHER_WATCHDOG_RESET";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_RESET";
    case ESP_RST_BROWNOUT: return "BROWNOUT_RESET";
    case ESP_RST_SDIO: return "SDIO_RESET";
    default: return "UNKNOWN_RESET";
  }
}

void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void sendData();
boolean reconnect();
void berechneUndSendeDruck();
void handleGPSData();
void manageWaterAndHeating();
void setHeatingPower();


void sendStartupMessage() {
  if (client.connected()) {
    String message;
    message.reserve(50);  // Reserve memory for the maximum expected size
    message = "ESP32 has rebooted. Reason: ";
    message += resetReason(esp_reset_reason());
    client.publish("/esp32/startup", message.c_str());
  }
}

void initializeSerial() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial1.begin(9600, SERIAL_8N1, 10, 11);
}

void initializeGPS() {
  GPS.begin(9600);                               // Baudrate des GPS-Empfängers
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // NMEA Ausgabe konfigurieren
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // Aktualisierungsrate auf 1Hz setzen
}

void initializeMHZ19() {
  mySerial.begin(9600, SERIAL_8N1, 44, 43);
  myMHZ19.begin(mySerial);
  myMHZ19.autoCalibration();
}

void initializeDS18B20() {
  sensors.begin();
  if (!sensors.getAddress(tempDeviceAddress, 0)) {
    Serial.println("Kein DS18B20 Sensor gefunden!");
  } else {
    Serial.println("DS18B20 Sensor gefunden und initialisiert.");
  }
}

void initializeMQTT() {
  client.setServer(mqtt_server, MQTT_PORT);
  client.setCallback(callback);
  if (client.connect("ESP32Client")) {
    sendStartupMessage();  // Senden Sie die Startnachricht
  }
}

void subscribeToTopics() {
  const char* topics[] = {
    "/relay/1", "/relay/2", "/relay/3", "/relay/4", "/relay/5", "/relay/6",
    "/ssr", "/ssrPower", "/MinDruck", "/Wasserversorgung", "/Boilerheizung",
    "/UVCLicht", "/Kuehlschrankgross", "/Kuehlschrankklein", "/Inverter"
  };

  for (const char* topic : topics) {
    client.subscribe(topic);
  }
}

void setup() {
  analogSetAttenuation(ADC_11db);

  initializeSerial();
  initializeGPS();
  initializeMHZ19();
  setup_wifi();
  initializeMQTT();

  pinMode(ssrPin, OUTPUT);
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ssrPin, 0);

  initializeDS18B20();

  MDNS.begin("esp32");
  server.begin();
  MDNS.addService("http", "tcp", OTA_PORT);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", WiFi.getHostname());

  subscribeToTopics();
  client.publish("/heartbeat", "setup");
}


void sendTemperature() {
  currentTemperature = getTemperature();
  if (currentTemperature != DEVICE_DISCONNECTED_C) {
    snprintf(tempMsg, sizeof(tempMsg) - 1, "%f", currentTemperature);
    client.publish("temperature", tempMsg);
  }
}

float getTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

void loop() {
  currentMillis = millis();

  // GPS-Daten verarbeiten
  handleGPSData();

  // Druck berechnen und senden, wenn geparkt
  if (isParked) {
    berechneUndSendeDruck();
  }

  // Wasser und Heizung verwalten
  manageWaterAndHeating();

  // OTA-Updates verarbeiten
  ArduinoOTA.handle();

  // MQTT-Verbindungsstatus überprüfen
  if (currentMillis - lastCheckTime >= CHECK_INTERVAL) {
    if (!client.connected()) {
      disconnectCount++;
      if (disconnectCount >= 3) {
        reconnect();
        disconnectCount = 0;  // Zurücksetzen des Zählers nach dem Reconnect-Versuch
      }
    } else {
      disconnectCount = 0;  // Zurücksetzen des Zählers, wenn die Verbindung besteht
    }
    lastCheckTime = currentMillis;  // Aktualisieren der letzten Überprüfungszeit
  }

  client.loop();

  // Temperaturdaten senden
  if (currentMillis - lastTempSentTime >= 20000) {  // 20 Sekunden
    currentTemperature = getTemperature();
    sendTemperature();
    lastTempSentTime = currentMillis;
  }

  // Heartbeat senden
  if (currentMillis - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    client.publish("/heartbeat", "alive");
    lastHeartbeatTime = currentMillis;
  }

  // CO2-Daten senden
  if (currentMillis - lastCO2SendTime >= 10000) {
    co2_ppm = myMHZ19.getCO2();  // Get CO2 concentration in ppm
    snprintf(co2Msg, sizeof(tempMsg) - 1, "%d", co2_ppm);
    client.publish("/CO2", co2Msg);
    lastCO2SendTime = currentMillis;
  }

  // Freien Heap-Speicher senden
  if (currentMillis - lastHeapSendTime >= HEAP_SEND_INTERVAL) {
    snprintf(heapMsg, sizeof(heapMsg), "%u", ESP.getFreeHeap());
    client.publish("/esp32/freeHeap", heapMsg);
    lastHeapSendTime = currentMillis;
  }
}

void setHeatingPower(int percentage) {
  pwmValue = map(percentage, 0, 100, 0, 255);
  ledcWrite(0, pwmValue);
}

void handleGPSData() {
  lastGPSCheckTime = 0;  // Zeitpunkt der letzten GPS-Datenverarbeitung
  lastGPSSyncTime = 0;   // Zeitpunkt der letzten GPS-Zeitsynchronisation
  if (currentMillis - lastGPSCheckTime >= 1000) {
    while (Serial1.available()) {
      c = GPS.read();

      if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA())) {
        currentSpeedKnots = GPS.speed;
        currentSpeedkmh = currentSpeedKnots * 1.852;

        if (currentSpeedkmh < 2) {  // Geschwindigkeitsgrenze in Knoten
          if (!isParked) {
            lastSendTime = currentMillis;
            isParked = true;
          } else if (currentMillis - lastSendTime >= 5000) {
            sendData();
            lastSendTime = currentMillis;
          }
        } else if (currentMillis - lastSendTime >= 1000 || isParked) {
          sendData();
          lastSendTime = currentMillis;
          isParked = false;
        }
      }
    }

    lastGPSCheckTime = currentMillis;  // Aktualisieren des Zeitpunkts der letzten GPS-Datenverarbeitung
  }
}

void manageWaterAndHeating() {
  lastExecutionTime = 0;  // Zeitpunkt der letzten Ausführung
  // Überprüfen, ob seit der letzten Ausführung 2 Sekunden vergangen sind
  if (currentMillis - lastExecutionTime >= 2000) {

    if (pressureCmFiltered <= MinDruck || !Wasserversorgung) {
      Boilerheizung = false;
      Wasserversorgung = false;
      if (relayStates[5]) {  // Überprüfen Sie, ob das Relais bereits ausgeschaltet ist
        sr.set(5, LOW);
        client.publish("/relay/6", "false");
        relayStates[5] = false;
      }
      setHeatingPower(0);
    } else {
      sr.set(5, HIGH);
      client.publish("/relay/6", "true");

      if (Boilerheizung) {
        client.publish("/Boilerheizung", "true");
        client.publish("/ssr", "true");
        if (millis() - heatingStartTime > 10000) {
          setHeatingPower(ssrPower);
        }
      }
    }

    lastExecutionTime = currentMillis;  // Aktualisieren des Zeitpunkts der letzten Ausführung
  }
}

void berechneUndSendeDruck() {
  // Zeitpunkt der letzten Ausführung
  lastExecutionTime1 = 0;

  // Überprüfen, ob seit der letzten Ausführung 2 Sekunden vergangen sind
  if (currentMillis - lastExecutionTime1 >= 2000) {
    sensorValue = analogRead(SENSOR_PIN);
    voltage = sensorValue * (3.3 / 4095.0);  // Umwandlung des ADC-Wertes in eine Spannung

    // Direkte Umrechnung der Spannung in Druck
    pressure_kPa = mapf(voltage, 0.0, 3.3, MIN_PRESSURE, MAX_PRESSURE);

    // Umrechnung von kPa in cm Wassersäule
    pressure_cm = pressure_kPa / 9.81;
    pressureCmFiltered = alpha * pressure_cm + (1.0 - alpha) * pressureCmFiltered;

    // Umrechnung von cm in Liter
    pressure_L = pressureCmFiltered * 3.2;

    if (isParked && abs(pressureCmFiltered - lastSentPressureCm) > 0.1 && currentMillis - lastPressureSentTime > 5000) {
      pressureMsg[50];
      snprintf(pressureMsg, sizeof(pressureMsg) - 1, "%f", pressure_L);
      client.publish("pressure", pressureMsg);
      lastSentPressureCm = pressureCmFiltered;
      lastPressureSentTime = currentMillis;
    }

    if (lastSentPressureCm < MinDruck) {
      waterPressureAlarm = true;
      client.publish("/wasserdruckalarm", "alarm");
    } else {
      waterPressureAlarm = false;
      client.publish("/wasserdruckalarm", "kein alarm");
    }

    lastExecutionTime1 = currentMillis;  // Aktualisieren des Zeitpunkts der letzten Ausführung
  }
}

void sendData() {
  kalmanGain = 0.4;
  filteredLatitude = previousLatitude + kalmanGain * (GPS.latitudeDegrees - previousLatitude);
  filteredLongitude = previousLongitude + kalmanGain * (GPS.longitudeDegrees - previousLongitude);
  isOutlier = abs(filteredLatitude - GPS.latitudeDegrees) > outlierThreshold || abs(filteredLongitude - GPS.longitudeDegrees) > outlierThreshold;
  previousLatitude = filteredLatitude;
  previousLongitude = filteredLongitude;

  if (!isOutlier) {
    snprintf(latMsg, sizeof(latMsg) - 1, "%f", filteredLatitude);
    snprintf(lonMsg, sizeof(lonMsg) - 1, "%f", filteredLongitude);
    snprintf(speedMsg, sizeof(speedMsg) - 1, "%f", currentSpeedkmh);
    snprintf(altMsg, sizeof(altMsg) - 1, "%f", GPS.altitude);

    client.publish("latitude", latMsg);
    client.publish("longitude", lonMsg);
    client.publish("speed", speedMsg);
    client.publish("altitude", altMsg);
  }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  startTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi.");
  }
}

void handleRelays(int index, bool state) {
  sr.set(index, state ? HIGH : LOW);
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';

  if (strncmp(topic, "/relay/", 7) == 0) {
    int relayIndex = topic[7] - '1';
    if (relayIndex >= 0 && relayIndex < 6) {
      relayStates[relayIndex] = (strcmp((char*)payload, "true") == 0);
      handleRelays(relayIndex, relayStates[relayIndex]);
      return;
    }
  }

  if (strcmp(topic, "/ssrPower") == 0) {
    ssrPower = atoi((char*)payload);
    Serial.println(ssrPower);
    return;
  }

  struct TopicAction {
    const char* topicName;
    int relayIndex;
    const char* logMessage;
  };

  TopicAction actions[] = {
    { "/Inverter", 1, "Setting Inverter relay" },
    { "/Kuehlschrankgross", 2, "Setting Kuehlschrankgross relay" },
    { "/Kuehlschrankklein", 3, nullptr },
    { "/UVCLicht", 4, nullptr },
    { "/Wasserversorgung", 5, nullptr },
    { "/Boilerheizung", -1, nullptr }
  };

  for (TopicAction action : actions) {
    if (strcmp(topic, action.topicName) == 0) {
      if (action.relayIndex != -1) {
        bool state = (strcmp((char*)payload, "true") == 0);
        sr.set(action.relayIndex, state ? HIGH : LOW);
        char relayTopic[15];
        snprintf(relayTopic, sizeof(relayTopic), "/relay/%d", action.relayIndex + 1);
        client.publish(relayTopic, state ? "true" : "false");
        relayStates[action.relayIndex] = state;
      }

      if (action.logMessage) {
        Serial.println(action.logMessage);
      }

      if (strcmp(topic, "/Boilerheizung") == 0) {
        bool state = (strcmp((char*)payload, "true") == 0);
        Boilerheizung = state;
        Wasserversorgung = state;
        client.publish("/Wasserversorgung", state ? "true" : "false");
        client.publish("/ssr", state ? "true" : "false");
        setHeatingPower(state ? ssrPower : 0);
      }

      return;
    }
  }

  if (!client.connected()) {
    char disconnectMessage[100];
    snprintf(disconnectMessage, sizeof(disconnectMessage), "Reconnected to MQTT broker. Last disconnect reason: %s", lastDisconnectReason);
    client.publish("/esp32/reconnect", disconnectMessage);
  }
}

boolean reconnect() {
  static unsigned long lastAttemptTime = 0;
  const unsigned long RECONNECT_INTERVAL = 10000;  // 5 Sekunden

  if (millis() - lastAttemptTime >= RECONNECT_INTERVAL) {
    lastAttemptTime = millis();

    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      return true;
    } else {
      Serial.println("Reconnect failed, trying again in 5 seconds.");
      return false;
    }
  }
  return false;
}
