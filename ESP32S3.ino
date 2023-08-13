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

const int SENSOR_PIN = 10;
const int MQTT_PORT = 1024;
const int OTA_PORT = 80;
const int SERIAL_BAUD_RATE = 115200;
const int GPS_BAUD_RATE = 9600;
const int RECONNECT_DELAY = 5000;

void callback(char* topic, byte* payload, unsigned int length);
void sendData();
boolean reconnect();
void berechneUndSendeDruck();
void setup_wifi();
void handleGPSData();


// Kalman Filter variables
double currentLatitude = 0.0;
double currentLongitude = 0.0;
double kalmanGain = 0.0;
double previousLatitude = 0.0;
double previousLongitude = 0.0;

const float SHUNT_RESISTOR = 150.0; // in Ohm
const float MIN_MA = 4.0;  // Minimum 4mA
const float MAX_MA = 20.0; // Maximum 20mA
const float MIN_PRESSURE = 0.0; // Minimum Druckwert (angepasst an Ihren Sensor)
const float MAX_PRESSURE = 100.0; // Maximum Druckwert (angepasst an Ihren Sensor)

// Outlier detection variables
const double outlierThreshold = 10.0;  // Adjust the threshold as needed
bool isOutlier = false;

// Replace with your network credentials
const char* ssid = "SSID";
const char* password = "PASSWORD";

unsigned long lastSendTime = 0;
bool isParked = false;

// Replace with your MQTT Broker address
const char* mqtt_server = "192.168.0.1";

// Shift register pins
const int SER_Pin = 7;    //pin 14 on the 75HC595
const int RCLK_Pin = 5;   //pin 12 on the 75HC595
const int SRCLK_Pin = 6;  //pin 11 on the 75HC595

// Number of shift registers
const int numOfShiftRegisters = 1;

// Additional SSR pin
const int ssrPin = 12;

// The serial connection to the GPS device
SoftwareSerial ss(11, 13);  // RX, TX

TinyGPSPlus gps;

// Create a ShiftRegister object
ShiftRegister74HC595<numOfShiftRegisters> sr(SER_Pin, RCLK_Pin, SRCLK_Pin);

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi();

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    ss.begin(GPS_BAUD_RATE);
    setup_wifi();
    client.setServer(mqtt_server, MQTT_PORT);
    client.setCallback(callback);
    pinMode(ssrPin, OUTPUT);

    // OTA setup
    MDNS.begin("esp32");
    server.begin();
    MDNS.addService("http", "tcp", OTA_PORT);
    Serial.printf("HTTPUpdateServer bereit! Öffnen Sie http://%s.local/update in Ihrem Browser\n", WiFi.getHostname());
}


void loop() {
    handleGPSData();
    berechneUndSendeDruck();
    ArduinoOTA.handle();
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}

void handleGPSData() {
    while (ss.available() > 0) {
        gps.encode(ss.read());
        if (gps.location.isUpdated()) {
            handleFahrzeugStatus();
        }
    }
}

void handleFahrzeugStatus() {
    // Überprüfen Sie, ob das Fahrzeug geparkt ist
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

void berechneUndSendeDruck() {
    int sensorValue = analogRead(SENSOR_PIN);
    float voltage = sensorValue * (3.3 / 4095.0);
    float current = voltage / SHUNT_RESISTOR;
    float pressure = MIN_PRESSURE + ((current - MIN_MA) / (MAX_MA - MIN_MA)) * (MAX_PRESSURE - MIN_PRESSURE);

    static float lastPressure = 0.0;
    if (isParked && abs(pressure - lastPressure) > 1.0) {  // Änderung von 1.0 als Beispiel für eine "wesentliche" Änderung
        Serial.println(pressure);
        char pressureMsg[50];
        snprintf(pressureMsg, 50, "%f", pressure);
        client.publish("pressure", pressureMsg);
        lastPressure = pressure;
    }
}

void sendData() {
  // Apply Kalman Filter
  kalmanGain = 0.4;  // Adjust the Kalman gain as needed
  double filteredLatitude = previousLatitude + kalmanGain * (gps.location.lat() - previousLatitude);
  double filteredLongitude = previousLongitude + kalmanGain * (gps.location.lng() - previousLongitude);

  // Check for outliers
  isOutlier = false;
  double latitudeDifference = abs(filteredLatitude - gps.location.lat());
  double longitudeDifference = abs(filteredLongitude - gps.location.lng());
  if (latitudeDifference > outlierThreshold || longitudeDifference > outlierThreshold) {
    isOutlier = true;
  }

  // Update previous values for the next iteration
  previousLatitude = filteredLatitude;
  previousLongitude = filteredLongitude;

  // Send data only if it is not an outlier
  if (!isOutlier) {
    char latMsg[50];
    snprintf(latMsg, 50, "%f", filteredLatitude);
    client.publish("latitude", latMsg);

    char lonMsg[50];
    snprintf(lonMsg, 50, "%f", filteredLongitude);
    client.publish("longitude", lonMsg);
  }

  // Send Speed
  char speedMsg[50];
  snprintf(speedMsg, 50, "%f", gps.speed.kmph());
  client.publish("speed", speedMsg);

  // Send Altitude
  char altMsg[50];
  snprintf(altMsg, 50, "%f", gps.altitude.meters());
  client.publish("altitude", altMsg);
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Verbindung zu ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi verbunden");
    Serial.println("IP-Adresse: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }

  // If a message is received on the topic, you check if the message is either "on" or "off".
  for (int i = 0; i < 8; i++) {
    String relayTopic = "/relay/" + String(i + 1);
    if (String(topic) == relayTopic) {
      if (messageTemp == "on") {
        sr.set(i, HIGH);
      } else if (messageTemp == "off") {
        sr.set(i, LOW);
      }
    }
  }

  // For SSR
  if (String(topic) == "/ssr") {
    if (messageTemp == "on") {
      digitalWrite(ssrPin, HIGH);
    } else if (messageTemp == "off") {
      digitalWrite(ssrPin, LOW);
    }
  }
}

boolean reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      for (int i = 0; i < 8; i++) {
        String relayTopic = "/relay/" + String(i + 1);
        client.publish(relayTopic.c_str(), "hello world");
        client.subscribe(relayTopic.c_str());
      }

      // For SSR
      client.publish("/ssr", "off");
      client.subscribe("/ssr");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

  // If MQTT connection is lost, turn off SSR
  if (!client.connected()) {
    digitalWrite(ssrPin, LOW);
  }

  return client.connected();
}
