#include <WiFi.h>
#include <PubSubClient.h>
#include <ShiftRegister74HC595.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Preferences.h>

WebServer server(80);
Preferences preferences;


// Constants
const int SENSOR_PIN = 14, MQTT_PORT = 1024, OTA_PORT = 80, SERIAL_BAUD_RATE = 115200, GPS_BAUD_RATE = 9600;
const int SER_Pin = 7, RCLK_Pin = 5, SRCLK_Pin = 6, numOfShiftRegisters = 1, ssrPin = 12;
const int PWM_FREQUENCY = 1000, PWM_RESOLUTION = 8;
const float SHUNT_RESISTOR = 150.0, MIN_MA = 4.0, MAX_MA = 20.0, MIN_PRESSURE = 0.0, MAX_PRESSURE = 100.0;
const double outlierThreshold = 10.0;
const float alpha = 0.1; // Dies ist der Glättungsfaktor. Werte näher bei 1 bedeuten weniger Glättung, Werte näher bei 0 bedeuten mehr Glättung.
static float pressureCmFiltered = 0; // Dieser Wert speichert den gefilterten Druck.
static float lastSentPressureCm = 0.0;
unsigned long lastPressureSentTime = 0;


// Variables
double currentLatitude = 0.0, currentLongitude = 0.0, kalmanGain = 0.0, previousLatitude = 0.0, previousLongitude = 0.0;
float pressure = 0.0, MinDruck = 2.0, pressure_cm = 0.0;
bool Wasserversorgung = false, Boilerheizung = false, UVCLicht = false, isOutlier = false, isParked = false;
unsigned long lastSendTime = 0;
bool relayStates[6] = {false, false, false, false, false, false};
bool ssrState = false;
int ssrPower = 0;
bool waterPressureAlarm = false; // Zu Beginn des Programms, kurz nach den anderen Variablen


// Network credentials
const char* ssid = "ssie";
const char* password = "password";
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
void manageWaterAndHeating();
void setHeatingPower();
void handleRoot();

void setup() {
    analogSetAttenuation(ADC_11db);
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
    client.subscribe("/ssrPower");
    client.publish("/MinDruck", String(MinDruck).c_str());
    client.subscribe("/MinDruck");
    client.publish("/Wasserversorgung", Wasserversorgung ? "on" : "off");
    client.subscribe("/Wasserversorgung");
    client.publish("/Boilerheizung", Boilerheizung ? "on" : "off");
    client.subscribe("/Boilerheizung");
    client.publish("/UVCLicht", UVCLicht ? "on" : "off");
    client.subscribe("/UVCLicht");

    preferences.begin("settings", false); // Öffnen Sie den Namespace "settings" im Lese-/Schreibmodus
    ssid = preferences.getString("ssid", "Wolf Verschwindibus").c_str();
    password = preferences.getString("password", "Philipp22121982").c_str();
    MinDruck = preferences.getFloat("MinDruck", 2.0);

    // Webserver-Endpunkte
    server.on("/", handleRoot);
    server.on("/save-settings", handleSaveSettings);
    server.begin();
    

    

}

void handleSaveSettings() {
    if (server.hasArg("ssid") && server.hasArg("password")) {
        ssid = server.arg("ssid").c_str();
        password = server.arg("password").c_str();
    }
    if (server.hasArg("MinDruck")) {
        MinDruck = server.arg("MinDruck").toFloat();
    }
    for (int i = 0; i < 6; i++) {
        String relayArg = "relay" + String(i + 1);
        if (server.hasArg(relayArg)) {
            relayStates[i] = server.arg(relayArg) == "on";
        }
    }
    if (server.hasArg("ssr")) {
        ssrState = server.arg("ssr") == "on";
    }
    if (server.hasArg("ssrPower")) {
        ssrPower = server.arg("ssrPower").toInt();
    }

    preferences.putString("ssid", server.arg("ssid"));
    preferences.putString("password", server.arg("password"));
    preferences.putFloat("MinDruck", MinDruck);

    server.send(200, "text/plain", "Einstellungen gespeichert!");

}

const char* settingsPage = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>Einstellungen</title>
    <!-- Bootstrap CSS -->
    <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/4.5.2/css/bootstrap.min.css">
    <!-- Optional JavaScript -->
    <!-- jQuery first, then Popper.js, then Bootstrap JS -->
    <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.5.1/jquery.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.16.0/umd/popper.min.js"></script>
    <script src="https://maxcdn.bootstrapcdn.com/bootstrap/4.5.2/js/bootstrap.min.js"></script>
</head>
<body>
    <div class="container mt-4">
        <h2>Einstellungen</h2>
        <form method="POST" action="/save-settings">
            <div class="form-group">
                <label for="ssid">SSID:</label>
                <input type="text" class="form-control" name="ssid" value="{ssid}">
            </div>
            <div class="form-group">
                <label for="password">Passwort:</label>
                <input type="password" class="form-control" name="password" value="{password}">
            </div>
            <div class="form-group">
                <label for="MinDruck">MinDruck:</label>
                <input type="text" class="form-control" name="MinDruck" value="{MinDruck}">
            </div>
            <!-- Weitere Formularelemente hier -->
            <button type="submit" class="btn btn-primary">Speichern</button>
        </form>
    </div>
</body>
</html>
)=====";


void loop() {
    handleGPSData();
    if (isParked) {
        berechneUndSendeDruck();
    }
    manageWaterAndHeating();
    ArduinoOTA.handle();
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    preferences.end(); // Schließen Sie die Preferences am Ende des loop()

}

void setHeatingPower(int percentage) {
    int pwmValue = map(percentage, 0, 100, 0, 255);
    ledcWrite(0, pwmValue);
}

void manageWaterAndHeating() {
    if (pressureCmFiltered <= MinDruck || !Wasserversorgung) {
        Boilerheizung = false;
        Wasserversorgung = false;
        if (relayStates[5]) { // Überprüfen Sie, ob das Relais bereits ausgeschaltet ist
            sr.set(5, LOW);
            client.publish("/relay/6", "off");
            relayStates[5] = false;
        }
        setHeatingPower(0);
    } else {
        sr.set(5, HIGH);
        client.publish("/relay/6", "on");
        if (Boilerheizung) {
            static unsigned long heatingStartTime = millis();
            if (millis() - heatingStartTime > 10000) {
                setHeatingPower(ssrPower);
                client.publish("/Boilerheizung", "on");
                client.publish("/ssr", "on");
                heatingStartTime = millis();
            }
        } else {
            setHeatingPower(0);
            client.publish("/ssr", "off");
        }
    }
}





void handleGPSData() {
    while (ss.available() > 0) {
        if (gps.encode(ss.read()) && gps.location.isUpdated()) {
            unsigned long currentMillis = millis();
            if (gps.speed.kmph() < 5.0) {
                if (!isParked) {
                    lastSendTime = currentMillis;
                    isParked = true;
                } else if (currentMillis - lastSendTime >= 1800000) {
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
}

void berechneUndSendeDruck() {
    int sensorValue = analogRead(SENSOR_PIN);
    sleep(2);
    float voltage = sensorValue * (3.0 / 4095.0);
    float current = voltage / SHUNT_RESISTOR * 1000;
    float pressure_kPa = MIN_PRESSURE + ((current - MIN_MA) / (MAX_MA - MIN_MA)) * (MAX_PRESSURE - MIN_PRESSURE);

    // Umrechnung von kPa in cm Wassersäule
    float pressure_cm = pressure_kPa * 10 / 9.81;
    pressureCmFiltered = alpha * pressure_cm + (1.0 - alpha) * pressureCmFiltered;

    // Umrechnung von cm in Liter
    float pressure_L = pressureCmFiltered * 3.2;

    unsigned long currentMillis = millis();
    if (isParked && abs(pressureCmFiltered - lastSentPressureCm) > 1.0 && currentMillis - lastPressureSentTime > 5000) {
        Serial.println(pressure_L);
        char pressureMsg[50];
        snprintf(pressureMsg, 50, "%f", pressure_L);
        client.publish("pressure", pressureMsg);
        lastSentPressureCm = pressureCmFiltered;
        lastPressureSentTime = currentMillis;
    }

    if (lastSentPressureCm < MinDruck && !waterPressureAlarm) { 
        waterPressureAlarm = true; 
        client.publish("/wasserdruckalarm", "alarm");
    } else if (lastSentPressureCm >= MinDruck && waterPressureAlarm) {
        waterPressureAlarm = false;
        client.publish("/wasserdruckalarm", "kein alarm");
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
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
        delay(500); // Hier können wir delay beibehalten, da es nur während der Einrichtung aufgerufen wird und keine anderen Aufgaben blockiert.
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Failed to connect to WiFi.");
    }
}


void handleRelays(int index, const char* payload) {
    sr.set(index, strcmp(payload, "on") == 0 ? HIGH : LOW);
}

void callback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';  // Füge ein Nullzeichen am Ende der Payload hinzu, um sie in einen gültigen C-String zu konvertieren
    String strPayload = String((char*)payload);  // Konvertiere Payload in einen Arduino String

    if (strncmp(topic, "/relay/", 7) == 0) {
        int relayIndex = topic[7] - '1';
        if (relayIndex >= 0 && relayIndex < 6) {
            relayStates[relayIndex] = strcmp((char*)payload, "on") == 0;
            handleRelays(relayIndex, (char*)payload);
            return;
        }
    }

    if (String(topic) == "/UVCLicht") {
        if (strPayload == "true") {
            sr.set(4, HIGH);  // Setze das 5. Relais (0-indexed) auf HIGH
            client.publish("/relay/5", "true");
            relayStates[4] = true;
        } else if (strPayload == "false") {
            sr.set(4, LOW);  // Setze das 5. Relais (0-indexed) auf LOW
            client.publish("/relay/5", "false");
            relayStates[4] = false;
        }
    }

    if (String(topic) == "/Wasserversorgung") {
        if (strPayload == "true") {
            Wasserversorgung = true;
        } else if (strPayload == "false") {
            Wasserversorgung = false;
            client.publish("/relay/6", "false");
            sr.set(5, LOW);
            client.publish("/Boilerheizung", "false");
            Boilerheizung = false;
        }
    }

    if (String(topic) == "/Boilerheizung") {
        if (strPayload == "true") {
            Boilerheizung = true;
            Wasserversorgung = true;
            client.publish("/Wasserversorgung", "true");
        } else if (strPayload == "false") {
            Boilerheizung = false;
            client.publish("/ssr", "false");
        }
    }

    const char* topicStr = (char*)topic;
    if (strcmp(topicStr, "/ssrPower") == 0) {
        ssrPower = atoi((char*)payload);
    } else if (strcmp(topicStr, "/ssr") == 0) {
        ssrState = strcmp((char*)payload, "true") == 0;
        setHeatingPower(ssrState ? 100 : 0);
    } else if (strcmp(topicStr, "/Wasserversorgung") == 0) {
        Wasserversorgung = strcmp((char*)payload, "true") == 0;
    } else if (strcmp(topicStr, "/Boilerheizung") == 0) {
        Boilerheizung = strcmp((char*)payload, "true") == 0;
    } else if (strcmp(topicStr, "/UVCLicht") == 0) {
        UVCLicht = strcmp((char*)payload, "true") == 0;
    } else if (strcmp(topicStr, "MinDruck") == 0) {
        MinDruck = atof((char*)payload);
    }
}

void handleRoot() {
    String html = "<html><body>";
    html += "<h1>Einstellungen</h1>";

    html += "<h2>Relais Zustände</h2>";
    for (int i = 0; i < 6; i++) {
        html += "Relais " + String(i + 1) + ": " + (relayStates[i] ? "ON" : "OFF") + "<br>";
    }

    html += "<h2>SSR Zustand</h2>";
    html += "SSR: " + String(ssrState ? "ON" : "OFF") + "<br>";
    html += "SSR Power: " + String(ssrPower) + "%<br>";

    html += "<h2>Wasservolumen</h2>";
    html += "Aktuelles Wasservolumen: " + String(pressureCmFiltered * 3.55) + " Liter<br>";

    html += "</body></html>";
    server.send(200, "text/html", html);
}

boolean reconnect() {
    unsigned long lastAttemptTime = millis();
    while (!client.connected()) {
        if (millis() - lastAttemptTime >= 5000) { // Ersetzt delay(5000)
            Serial.print("Attempting MQTT connection...");
            if (client.connect("ESP32Client")) {
                Serial.println("connected");
                // Abonnieren von relay Topics
                for (int i = 0; i < 6; i++) {
                    char relayTopic[10];
                    snprintf(relayTopic, sizeof(relayTopic), "/relay/%d", i + 1);
                    client.publish(relayTopic, "off");
                    client.subscribe(relayTopic);
                }
                client.publish("/ssrPower", "0");

                // Abonnieren von weiteren Topics
                client.publish("/ssr", "off");
                client.subscribe("/ssr");
                client.subscribe("/ssrPower");
                client.subscribe("/MinDruck");
                client.subscribe("/Wasserversorgung");
                client.subscribe("/Boilerheizung");
                client.subscribe("/UVCLicht");
                return true;
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
                lastAttemptTime = millis();
            }
        }
    }
    digitalWrite(ssrPin, LOW);
    return false;
}
