#include <WiFi.h>
#include <PubSubClient.h>
#include <ShiftRegister74HC595.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>

WebServer server(80);

// Constants
const int SENSOR_PIN = 10, MQTT_PORT = 1024, OTA_PORT = 80, SERIAL_BAUD_RATE = 115200, GPS_BAUD_RATE = 9600;
const int SER_Pin = 7, RCLK_Pin = 5, SRCLK_Pin = 6, numOfShiftRegisters = 1, ssrPin = 12;
const int PWM_FREQUENCY = 1000, PWM_RESOLUTION = 8;
const float SHUNT_RESISTOR = 150.0, MIN_MA = 4.0, MAX_MA = 20.0, MIN_PRESSURE = 0.0, MAX_PRESSURE = 100.0;
const double outlierThreshold = 10.0;

// Variables
double currentLatitude = 0.0, currentLongitude = 0.0, kalmanGain = 0.0, previousLatitude = 0.0, previousLongitude = 0.0;
float pressure = 0.0, MinDruck = 0.0, TankLimitDruck = 0.05;
bool Wasserversorgung = false, Boilerheizung = false, UVCLicht = false, isOutlier = false, isParked = false;
unsigned long lastSendTime = 0;

// Network credentials
const char* ssid = "SSID";
const char* password = "PASSWORD";
const char* mqtt_server = "192.168.0.1";

SoftwareSerial ss(11, 13);
TinyGPSPlus gps;
ShiftRegister74HC595<numOfShiftRegisters> sr(SER_Pin, RCLK_Pin, SRCLK_Pin);
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void sendData();
boolean reconnect();
void berechneUndSendeDruck();
void handleGPSData();

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    ss.begin(GPS_BAUD_RATE);
    setup_wifi();
    client.setServer(mqtt_server, MQTT_PORT);
    client.setCallback(callback);
    pinMode(ssrPin, OUTPUT);
    ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(ssrPin, 0);

    MDNS.begin("esp32");
    server.begin();
    MDNS.addService("http", "tcp", OTA_PORT);
    Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", WiFi.getHostname());
    
    for (int i = 0; i < 6; i++) {
        char relayTopic[10];
        snprintf(relayTopic, sizeof(relayTopic), "/relay/%d", i + 1);
        client.publish(relayTopic, "off");
        client.subscribe(relayTopic);
    }
    client.publish("/ssr", "off");
    client.subscribe("/ssr");
    client.publish("/ssrPower", "0");
    client.publish("/TankLimitDruck", String(TankLimitDruck).c_str());
    client.publish("/Wasserversorgung", Wasserversorgung ? "on" : "off");
    client.publish("/Boilerheizung", Boilerheizung ? "on" : "off");
    client.publish("/UVCLicht", UVCLicht ? "on" : "off");
}

void loop() {
    handleGPSData();
    berechneUndSendeDruck();
    if (pressure <= TankLimitDruck || !Wasserversorgung) {
        Boilerheizung = false;
        Wasserversorgung = false;
        sr.set(5, LOW);
        setHeatingPower(0);
    }
    sr.set(5, Wasserversorgung ? HIGH : LOW);
    if (Boilerheizung) {
        sr.set(5, HIGH);
        delay(10000);
        setHeatingPower(100);
    } else {
        setHeatingPower(0);
    }
    ArduinoOTA.handle();
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}

void setHeatingPower(int percentage) {
    int pwmValue = map(percentage, 0, 100, 0, 255);
    ledcWrite(0, pwmValue);
}

void handleGPSData() {
    while (ss.available() > 0) {
        gps.encode(ss.read());
        if (gps.location.isUpdated()) {
            if (gps.speed.kmph() < 2.0) {
                if (!isParked) {
                    lastSendTime = millis();
                    isParked = true;
                } else if (millis() - lastSendTime >= 1800000) {
                    sendData();
                    lastSendTime = millis();
                }
            } else if (millis() - lastSendTime >= 1000 || isParked) {
                sendData();
                lastSendTime = millis();
                isParked = false;
            }
        }
    }
}

void berechneUndSendeDruck() {
    int sensorValue = analogRead(SENSOR_PIN);
    float voltage = sensorValue * (3.3 / 4095.0);
    float current = voltage / SHUNT_RESISTOR;
    pressure = MIN_PRESSURE + ((current - MIN_MA) / (MAX_MA - MIN_MA)) * (MAX_PRESSURE - MIN_PRESSURE);
    static float lastPressure = 0.0;
    if (isParked && abs(pressure - lastPressure) > 1.0) {
        Serial.println(pressure);
        char pressureMsg[50];
        snprintf(pressureMsg, 50, "%f", pressure);
        client.publish("pressure", pressureMsg);
        lastPressure = pressure;
    }
}

void sendData() {
    kalmanGain = 0.4;
    double filteredLatitude = previousLatitude + kalmanGain * (gps.location.lat() - previousLatitude);
    double filteredLongitude = previousLongitude + kalmanGain * (gps.location.lng() - previousLongitude);
    isOutlier = abs(filteredLatitude - gps.location.lat()) > outlierThreshold || abs(filteredLongitude - gps.location.lng()) > outlierThreshold;
    previousLatitude = filteredLatitude;
    previousLongitude = filteredLongitude;
    if (!isOutlier) {
        char latMsg[50], lonMsg[50], speedMsg[50], altMsg[50];
        snprintf(latMsg, 50, "%f", filteredLatitude);
        snprintf(lonMsg, 50, "%f", filteredLongitude);
        snprintf(speedMsg, 50, "%f", gps.speed.kmph());
        snprintf(altMsg, 50, "%f", gps.altitude.meters());
        client.publish("latitude", latMsg);
        client.publish("longitude", lonMsg);
        client.publish("speed", speedMsg);
        client.publish("altitude", altMsg);
    }
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void handleRelays(int index, const char* payload) {
    sr.set(index, strcmp(payload, "on") == 0 ? HIGH : LOW);
}

void callback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    if (strncmp(topic, "/relay/", 7) == 0) {
        int relayIndex = topic[7] - '1';
        if (relayIndex >= 0 && relayIndex < 6) {
            handleRelays(relayIndex, (char*)payload);
            return;
        }
    }
    const char* topicStr = (char*)topic;
    if (strcmp(topicStr, "/ssrPower") == 0) {
        setHeatingPower(atoi((char*)payload));
    } else if (strcmp(topicStr, "/ssr") == 0) {
        setHeatingPower(strcmp((char*)payload, "on") == 0 ? 100 : 0);
    } else if (strcmp(topicStr, "/Wasserversorgung") == 0) {
        Wasserversorgung = strcmp((char*)payload, "on") == 0;
    } else if (strcmp(topicStr, "/Boilerheizung") == 0) {
        Boilerheizung = strcmp((char*)payload, "on") == 0;
    } else if (strcmp(topicStr, "/UVCLicht") == 0) {
        UVCLicht = strcmp((char*)payload, "on") == 0;
    } else if (strcmp(topicStr, "/MinDruck") == 0) {
        MinDruck = atof((char*)payload);
    }
}

boolean reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected");
            for (int i = 0; i < 6; i++) {
                char relayTopic[10];
                snprintf(relayTopic, sizeof(relayTopic), "/relay/%d", i + 1);
                client.publish(relayTopic, "off");
                client.subscribe(relayTopic);
            }
            client.publish("/ssr", "off");
            client.subscribe("/ssr");
            return true;
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
    digitalWrite(ssrPin, LOW);
    return false;
}
