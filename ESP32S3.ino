// 1. Bibliotheken importieren
#include <WiFi.h>
#include <PubSubClient.h>
#include <ShiftRegister74HC595.h>
#include <TinyGPS++.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MHZ19.h"
#include <array>
#include <unordered_map>
#include <functional>

MHZ19 myMHZ19;
HardwareSerial mySerial(2);

// 2. Konstanten definieren
constexpr int DS18B20_PIN = 9;
constexpr int SER_Pin = 7;
constexpr int RCLK_Pin = 5;
constexpr int SRCLK_Pin = 6;
constexpr int numOfShiftRegisters = 1;
WebServer server(80);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
TinyGPSPlus GPS;
ShiftRegister74HC595<numOfShiftRegisters> sr(SER_Pin, RCLK_Pin, SRCLK_Pin);
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long heatingStartTime, lastExecutionTime, lastGPSCheckTime, lastGPSSyncTime, currentMillis, lastExecutionTime1, currentTemperature, lastSendTime, lastExecutionTimeDruck, lastPressureSentTime, lastHeartbeatTime, lastCO2SendTime, lastCheckTime, lastHeapSendTime, lastTempSentTime, startTime;
double filteredLatitude, filteredLongitude, previousLatitude, previousLongitude, kalmanGain;
float currentSpeedkmh, pressure, MinDruck, pressure_cm, pressureCmFiltered = 0, lastSentPressureCm = 0.0, voltage, pressure_kPa, pressure_L;
std::array<bool, 6> relayStates = { false, false, false, false, false, false };
bool Wasserversorgung = false, Boilerheizung = false, UVCLicht = false, isOutlier = false, isParked = false, Kuehlschrankgross, Kuehlschrankklein, Inverter, ssrState = false, waterPressureAlarm = false, state;
int disconnectCount = 0, ssrPower = 100, sensorValue, pwmValue, co2_ppm;
std::array<char, 50> tempMsg, heapMsg, latMsg, lonMsg, speedMsg, altMsg, co2Msg;
std::array<char, 50> pressureMsg = { 0 };
const char* topicStr;
//char c;
const char* lastDisconnectReason = "Unknown";

// 4. Netzwerkkonfiguration
char ssid[] = "Wolf Verschwindibus";
char password[] = "Philipp22121982";
const char* mqtt_server = "192.168.0.1";
const char* command = "PMTK353,1,1,0,0,0";
unsigned char checksum = 0;
constexpr int SENSOR_PIN = 14;
constexpr int MQTT_PORT = 1024;
constexpr int OTA_PORT = 80;
constexpr int SERIAL_BAUD_RATE = 115200;
constexpr int GPS_BAUD_RATE = 9600;

constexpr int ssrPin = 13;
constexpr int PWM_FREQUENCY = 1000;
constexpr int PWM_RESOLUTION = 8;
constexpr float MIN_PRESSURE = 0.0;
constexpr float MAX_PRESSURE = 100.0;
constexpr double outlierThreshold = 10.0;
constexpr float alpha = 0.2;
constexpr unsigned long HEARTBEAT_INTERVAL = 2000;
constexpr unsigned long HEAP_SEND_INTERVAL = 10000;
constexpr unsigned long CHECK_INTERVAL = 10000;
constexpr char TRUE_STR[] = "true";
constexpr char FALSE_STR[] = "false";
constexpr char relayTopics[][9] = {"/relay/1", "/relay/2", "/relay/3", "/relay/4", "/relay/5", "/relay/6"};
constexpr char PMTK_SET_NMEA_OUTPUT_RMCGNS[] = "$PMTK314,1,1,5,1,5,5,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
constexpr int MIN_SATELLITES = 6;
constexpr int MAX_HDOP = 150;
constexpr float SPEED_THRESHOLD = 2.0f;
constexpr unsigned long PARKED_SEND_INTERVAL = 5000;
constexpr unsigned long MOVING_SEND_INTERVAL = 1000;
constexpr int MAX_GPS_ITERATIONS = 10;
constexpr unsigned long EXECUTION_INTERVAL = 2000;
constexpr unsigned long HEATING_START_INTERVAL = 10000;
constexpr double ADC_MAX_VALUE = 4095.0;
constexpr double ADC_VOLTAGE = 3.3;
constexpr double GRAVITY_ACCELERATION = 9.81;
constexpr double KALMAN_GAIN = 0.4;
inline void handleTopicAction(int relayIndex, const char* logMessage, const char* payload) {
    bool localState = (strcmp(payload, TRUE_STR) == 0);
    
    // Check if the state has changed
    if (relayStates[relayIndex] != localState) {
        // Set relay state
        sr.set(relayIndex, localState ? HIGH : LOW);
        relayStates[relayIndex] = localState;

        // Publish relay topic
        client.publish(relayTopics[relayIndex], localState ? TRUE_STR : FALSE_STR);

        // Log message if provided
        if (logMessage) {
        }
    }
}
using Handler = std::function<void(const char*)>;

    static std::unordered_map<std::string, Handler> handlers = {
        {"/ssrPower", [](const char* payload) {
            ssrPower = atoi(payload);
        }},
        {"/Inverter", [](const char* payload) {
            handleTopicAction(1, "Setting Inverter relay", payload);
        }},
        {"/Kuehlschrankgross", [](const char* payload) {
            handleTopicAction(2, "Setting Kuehlschrankgross relay", payload);
        }},
        {"/Kuehlschrankklein", [](const char* payload) {
            handleTopicAction(3, nullptr, payload);
        }},
        {"/UVCLicht", [](const char* payload) {
            handleTopicAction(4, nullptr, payload);
        }},
        {"/Wasserversorgung", [](const char* payload) {
            handleTopicAction(5, nullptr, payload);
        }},
        {"/Boilerheizung", [](const char* payload) {
            state = (strcmp(payload, "true") == 0);
            Boilerheizung = state;
            Wasserversorgung = state;
            client.publish("/Wasserversorgung", state ? "true" : "false");
            client.publish("/ssr", state ? "true" : "false");
            setHeatingPower(state ? ssrPower : 0);
        }},
        {"/MinDruck", [](const char* payload) {  // Hinzugefügter Handler für MinDruck
            MinDruck = atof(payload);
        }},
        {"/calibrateCO2", [](const char* payload) {
            if (strcmp(payload, "true") == 0) {
                calibrateCO2Sensor();
            }
        }}
    };

// 3. Globale Variablen definieren



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
    char message[100];
snprintf(message, sizeof(message), "ESP32 has rebooted. Reason: %s", resetReason(esp_reset_reason()));
client.publish("/esp32/startup", message);
  }
}

void initializeSerial() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial1.begin(9600, SERIAL_8N1, 10, 11);
}

void initializeMHZ19() {
  mySerial.begin(9600, SERIAL_8N1, 44, 43);
  myMHZ19.begin(mySerial);
  //myMHZ19.autoCalibration();
}

void initializeDS18B20() {
  sensors.begin();
  if (!sensors.getAddress(tempDeviceAddress, 0)) {
  } else {
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
  std::array<const char*, 15> topics = {
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
  initializeMHZ19();
  setup_wifi();
  initializeMQTT();

  pinMode(ssrPin, OUTPUT);
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ssrPin, 0);

  initializeDS18B20();

  for (int i = 0; command[i]; i++) {
    checksum ^= command[i];
  }
  // Erstellen Sie den vollständigen Befehl mit der Checksumme
  char fullCommand[30];
  snprintf(fullCommand, sizeof(fullCommand), "$%s*%02X\r\n", command, checksum);
  // Senden Sie den Befehl an das GPS-Gerät
  Serial1.print(fullCommand);

  MDNS.begin("esp32");
  server.begin();
  MDNS.addService("http", "tcp", OTA_PORT);

  subscribeToTopics();
  client.publish("/heartbeat", "setup");
}


void sendTemperature() {
  currentTemperature = getTemperature();
  if (currentTemperature != DEVICE_DISCONNECTED_C) {
snprintf(tempMsg.data(), tempMsg.size() , "%f", currentTemperature);
client.publish("temperature", tempMsg.data());
  }
}

float getTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

void loop() {
  currentMillis = millis();

  handleGPSData();
  handlePressure();
  manageWaterAndHeating();
  handleOTAUpdates();
  checkMQTTConnection();
  handleMQTTLoop();
  sendTemperatureData();
  sendCO2Data();
  sendHeapData();
}

void handlePressure() {
  if (isParked) {
    berechneUndSendeDruck();
  }
}

void handleOTAUpdates() {
  ArduinoOTA.handle();
}

void checkMQTTConnection() {
  if (currentMillis - lastCheckTime >= CHECK_INTERVAL) {
    if (!client.connected()) {
      disconnectCount++;
      if (disconnectCount >= 3) {
        reconnect();
        disconnectCount = 0;
      }
    } else {
      disconnectCount = 0;
    }
    lastCheckTime = currentMillis;
  }
}

void handleMQTTLoop() {
  client.loop();
}

void sendTemperatureData() {
  if (currentMillis - lastTempSentTime >= 20000) {
    currentTemperature = getTemperature();
    sendTemperature();
    lastTempSentTime = currentMillis;
  }
}

void sendCO2Data() {
  if (currentMillis - lastCO2SendTime >= 10000) {
    co2_ppm = myMHZ19.getCO2();
    char co2Msg[10];  // Assuming CO2 ppm won't exceed 9999
    snprintf(co2Msg, sizeof(co2Msg), "%d", co2_ppm);
    client.publish("/CO2", co2Msg);
    lastCO2SendTime = currentMillis;
  }
}

void sendHeapData() {
  if (currentMillis - lastHeapSendTime >= HEAP_SEND_INTERVAL) {
    //char heapMsg[10];  // Assuming heap size won't exceed 9999999999 bytes
    snprintf(heapMsg.data(), heapMsg.size(), "%u", ESP.getFreeHeap());

    client.publish("/esp32/freeHeap", heapMsg.data());
    lastHeapSendTime = currentMillis;
  }
}

void setHeatingPower(int percentage) {
  pwmValue = map(percentage, 0, 100, 0, 255);
  ledcWrite(0, pwmValue);
}

void processGPSData() {
    if (GPS.location.isValid() && GPS.satellites.value() >= MIN_SATELLITES && GPS.hdop.value() <= MAX_HDOP) {
        currentSpeedkmh = GPS.speed.kmph();
        unsigned long timeSinceLastSend = currentMillis - lastSendTime;

        if (currentSpeedkmh < SPEED_THRESHOLD) {
            if (!isParked) {
                lastSendTime = currentMillis;
                isParked = true;
            } else if (timeSinceLastSend >= PARKED_SEND_INTERVAL) {
                sendData();
                lastSendTime = currentMillis;
            }
        } else if (timeSinceLastSend >= MOVING_SEND_INTERVAL || isParked) {
            sendData();
            lastSendTime = currentMillis;
            isParked = false;
        }
    }
}

void handleGPSData() {
    if (currentMillis - lastGPSCheckTime >= MOVING_SEND_INTERVAL) {
        int iteration = 0;
        while (Serial1.available() && iteration < MAX_GPS_ITERATIONS) {
            GPS.encode(Serial1.read());
            processGPSData();
            iteration++;
        }
        lastGPSCheckTime = currentMillis;
    }
}

void publishRelayState(const char* topic, bool state) {
    client.publish(topic, state ? "true" : "false");
}

void manageWaterAndHeating() {
    if (currentMillis - lastExecutionTime >= EXECUTION_INTERVAL) {
        if (pressureCmFiltered <= MinDruck || !Wasserversorgung) {
            Boilerheizung = false;
            Wasserversorgung = false;
            if (relayStates[5]) {
                sr.set(5, LOW);
                publishRelayState("/relay/6", false);
                relayStates[5] = false;
            }
            setHeatingPower(0);
        } else {
            sr.set(5, HIGH);
            publishRelayState("/relay/6", true);

            if (Boilerheizung) {
                publishRelayState("/Boilerheizung", true);
                publishRelayState("/ssr", true);
                if (currentMillis - heatingStartTime > HEATING_START_INTERVAL) {
                    setHeatingPower(ssrPower);
                }
            }
        }

        lastExecutionTime = currentMillis;
    }
}

void formatAndPublishPressure(const char* topic, double value) {
    char msg[50];
    snprintf(msg, sizeof(msg), "%f", value);
    client.publish(topic, msg);
}

double calculatePressure_kPa(double voltage) {
    return mapf(voltage, 0.0, ADC_VOLTAGE, MIN_PRESSURE, MAX_PRESSURE);
}

void berechneUndSendeDruck() {
    if (currentMillis - lastExecutionTime1 >= 2000) {
        sensorValue = analogRead(SENSOR_PIN);
        voltage = sensorValue * (ADC_VOLTAGE / ADC_MAX_VALUE);

        pressure_kPa = calculatePressure_kPa(voltage);
        pressure_cm = pressure_kPa / GRAVITY_ACCELERATION;
        pressureCmFiltered = alpha * pressure_cm + (1.0 - alpha) * pressureCmFiltered;
        pressure_L = pressureCmFiltered * 3.2;

        if (isParked && abs(pressureCmFiltered - lastSentPressureCm) > 0.1 && currentMillis - lastPressureSentTime > 5000) {
            formatAndPublishPressure("pressure", pressure_L);
            lastSentPressureCm = pressureCmFiltered;
            lastPressureSentTime = currentMillis;
        }

        const char* alarmStatus = (lastSentPressureCm < MinDruck) ? "alarm" : "kein alarm";
        client.publish("/wasserdruckalarm", alarmStatus);

        lastExecutionTime1 = currentMillis;
    }
}

void formatAndPublish(const char* topic, double value) {
    char msg[50];
    snprintf(msg, sizeof(msg), "%f", value);
    client.publish(topic, msg);
}

void sendData() {
    double latitudeError = GPS.location.lat() - previousLatitude;
    double longitudeError = GPS.location.lng() - previousLongitude;

    filteredLatitude = previousLatitude + KALMAN_GAIN * latitudeError;
    filteredLongitude = previousLongitude + KALMAN_GAIN * longitudeError;

    bool isLatitudeOutlier = abs(latitudeError) > outlierThreshold;
    bool isLongitudeOutlier = abs(longitudeError) > outlierThreshold;

    previousLatitude = filteredLatitude;
    previousLongitude = filteredLongitude;

    if (!isLatitudeOutlier && !isLongitudeOutlier) {
        formatAndPublish("latitude", filteredLatitude);
        formatAndPublish("longitude", filteredLongitude);
        formatAndPublish("speed", currentSpeedkmh);
    }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  startTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
  }

  if (WiFi.status() == WL_CONNECTED) {
    
    // IP-Adresse des ESP32 abrufen
    IPAddress ip = WiFi.localIP();
    char ipStr[16];  // Platz für die IP-Adresse im Format "xxx.xxx.xxx.xxx"
    snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    // IP-Adresse über MQTT senden
    client.publish("/esp32/ip_address", ipStr);
  } else {
  }
}

void calibrateCO2Sensor() {
    myMHZ19.calibrate();
}

void handleRelays(int index, bool state) {
  sr.set(index, state ? HIGH : LOW);
}

void callback(char* topic, byte* payload, unsigned int length) {
    char* payloadStr = reinterpret_cast<char*>(payload);
    payloadStr[length] = '\0';  // Null-terminate the payload

    std::string topicStr(topic);

    if (topicStr.substr(0, 7) == "/relay/") {
        int relayIndex = topic[7] - '1';
        if (relayIndex >= 0 && relayIndex < 6) {
            relayStates[relayIndex] = (strcmp(payloadStr, "true") == 0);
            handleRelays(relayIndex, relayStates[relayIndex]);
            return;
        }
    } else {
        auto handlerIt = handlers.find(topicStr);
        if (handlerIt != handlers.end()) {
            handlerIt->second(payloadStr);
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
      return true;
    } else {
      return false;
    }
  }
  return false;
}
