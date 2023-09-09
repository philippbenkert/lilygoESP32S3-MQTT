#include <WiFi.h>
#include <PubSubClient.h>
#include <ShiftRegister74HC595.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>

WebServer server(80);
Preferences preferences;


// Constants
const int SENSOR_PIN = 14, MQTT_PORT = 1024, OTA_PORT = 80, SERIAL_BAUD_RATE = 115200, GPS_BAUD_RATE = 9600;
const int SER_Pin = 7, RCLK_Pin = 5, SRCLK_Pin = 6, numOfShiftRegisters = 1, ssrPin = 13;
const int PWM_FREQUENCY = 1000, PWM_RESOLUTION = 8;
const float MIN_PRESSURE = 0.0, MAX_PRESSURE = 100.0;
const double outlierThreshold = 10.0;
const float alpha = 0.2; // Dies ist der Glättungsfaktor. Werte näher bei 1 bedeuten weniger Glättung, Werte näher bei 0 bedeuten mehr Glättung.
static float pressureCmFiltered = 0; // Dieser Wert speichert den gefilterten Druck.
static float lastSentPressureCm = 0.0;
unsigned long lastPressureSentTime = 0;
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
const int DS18B20_PIN = 9;
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; // Globale Deklaration der DS18B20-Adresse
float currentSpeedKnots = 0.0;

const unsigned long HEARTBEAT_INTERVAL = 1000; // 1 Sekunden
unsigned long lastHeartbeatTime = 0;


// Variables
double currentLatitude = 0.0, currentLongitude = 0.0, kalmanGain = 0.0, previousLatitude = 0.0, previousLongitude = 0.0;
float pressure = 0.0, MinDruck = 2.0, pressure_cm = 0.0;
bool Wasserversorgung = false, Boilerheizung = false, UVCLicht = false, isOutlier = false, isParked = false, Kuehlschrankgross, Kuehlschrankklein, Inverter;
unsigned long lastSendTime = 0;
bool relayStates[6] = {false, false, false, false, false, false};
bool ssrState = false;
int ssrPower = 100;
bool waterPressureAlarm = false; // Zu Beginn des Programms, kurz nach den anderen Variablen


// Network credentials
char ssid[] = "ssid";
char password[] = "password";
const char* mqtt_server = "192.168.0.1";

SoftwareSerial ss(10, 11);
Adafruit_GPS GPS(&ss);
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
    ss.begin(9600);  // Baudrate des GPS-Empfängers
    GPS.begin(9600);  // Baudrate des GPS-Empfängers
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // NMEA Ausgabe konfigurieren
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // Aktualisierungsrate auf 1Hz setzen
    setup_wifi();
    client.setServer(mqtt_server, MQTT_PORT);
    client.setCallback(callback);
    pinMode(ssrPin, OUTPUT);
    ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(ssrPin, 0);
    sensors.begin();
    if (!sensors.getAddress(tempDeviceAddress, 0)) {
        Serial.println("Kein DS18B20 Sensor gefunden!");
    } else {
        Serial.println("DS18B20 Sensor gefunden und initialisiert.");
    }
    uint8_t numberOfDevices = sensors.getDeviceCount();



    MDNS.begin("esp32");
    server.begin();
    MDNS.addService("http", "tcp", OTA_PORT);
    Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", WiFi.getHostname());
    
    for (int i = 0; i < 6; i++) {
        char relayTopic[10];
        snprintf(relayTopic, sizeof(relayTopic), "/relay/%d", i + 1);
        //client.publish(relayTopic, "off");
        client.subscribe(relayTopic);
    }
    client.publish("/ssr", "false");
    client.subscribe("/ssr");
    client.subscribe("/ssrPower");
    client.subscribe("/MinDruck");
    client.publish("/Wasserversorgung", "false");
    client.subscribe("/Wasserversorgung");
    client.publish("/Boilerheizung", "false");
    client.subscribe("/Boilerheizung");
    client.publish("/UVCLicht", "false");
    client.subscribe("/UVCLicht");
    client.subscribe("/Kuehlschrankgross");
    client.publish("/Kuehlschrankgross", "false");
    client.subscribe("/Kuehlschrankklein");
    client.publish("/Kuehlschrankklein", "false");
    client.subscribe("/Inverter");
    client.publish("/Inverter", "false");

    preferences.begin("settings", false); // Öffnen Sie den Namespace "settings" im Lese-/Schreibmodus
    String ssidPref = preferences.getString("ssid", "ssid");
    String passwordPref = preferences.getString("password", "password");
    strncpy(ssid, ssidPref.c_str(), sizeof(ssid) - 1);
    strncpy(password, passwordPref.c_str(), sizeof(password) - 1);
    MinDruck = preferences.getFloat("MinDruck", -20.0);

    // Webserver-Endpunkte
    server.on("/", handleRoot);
    server.on("/save-settings", handleSaveSettings);
    server.begin();
    

    

}

void sendTemperature() {
    float currentTemperature = getTemperature();
    if (currentTemperature == DEVICE_DISCONNECTED_C) {
    } else {
    }
    
    char tempMsg[50];
    snprintf(tempMsg, 50, "%f", currentTemperature);
    client.publish("temperature", tempMsg);
}


float getTemperature() {
    sensors.requestTemperatures(); 
    return sensors.getTempCByIndex(0); 
}

void handleSaveSettings() {
    if (server.hasArg("ssid")) {
        strncpy(ssid, server.arg("ssid").c_str(), sizeof(ssid) - 1);
        preferences.putString("ssid", server.arg("ssid"));
    }
    if (server.hasArg("password")) {
        strncpy(password, server.arg("password").c_str(), sizeof(password) - 1);
        preferences.putString("password", server.arg("password"));
    }
    if (server.hasArg("MinDruck")) {
        MinDruck = server.arg("MinDruck").toFloat();
    }
    for (int i = 0; i < 6; i++) {
        String relayArg = "relay" + String(i + 1);
        if (server.hasArg(relayArg)) {
            relayStates[i] = server.arg(relayArg) == "true";
        }
    }
    if (server.hasArg("ssr")) {
        ssrState = server.arg("ssr") == "true";
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
    float currentTemperature = getTemperature();
    static unsigned long lastTempSentTime = 0;

    if (millis() - lastTempSentTime >= 20000) { // 20 Sekunden
    sendTemperature();
    lastTempSentTime = millis();
    }

    if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
        client.publish("/heartbeat", "alive");
        lastHeartbeatTime = millis();
    }

}


}

void setHeatingPower(int percentage) {
    int pwmValue = map(percentage, 0, 100, 0, 255);
    ledcWrite(0, pwmValue);
    
    
}

void synchronizeTimeWithGPS() {
    if (GPS.year > 2000) { // Überprüfen Sie, ob das GPS-Datum gültig ist
        struct tm timeInfo;
        timeInfo.tm_year = GPS.year - 1900;
        timeInfo.tm_mon = GPS.month - 1;
        timeInfo.tm_mday = GPS.day;
        timeInfo.tm_hour = GPS.hour;
        timeInfo.tm_min = GPS.minute;
        timeInfo.tm_sec = GPS.seconds;
        time_t epochTime = mktime(&timeInfo);
        if (epochTime != -1) {
            struct timeval tv;
            tv.tv_sec = epochTime;
            tv.tv_usec = 0;
            settimeofday(&tv, NULL);
            Serial.println("Uhrzeit mit GPS synchronisiert!");
        }
    }
}


void handleGPSData() {
    static unsigned long lastGPSCheckTime = 0; // Zeitpunkt der letzten GPS-Datenverarbeitung
    unsigned long currentMillis = millis();
    synchronizeTimeWithGPS(); // Fügen Sie diese Zeile hinzu
    // Überprüfen, ob seit dem letzten Verarbeiten der GPS-Daten 2 Sekunden vergangen sind
    if (currentMillis - lastGPSCheckTime >= 2000) {
        while (ss.available()) {
            char c = GPS.read();

            if (GPS.newNMEAreceived()) {
                if (GPS.parse(GPS.lastNMEA())) {
                    currentSpeedKnots = GPS.speed;

                    if (currentSpeedKnots < 2) {  // Geschwindigkeitsgrenze in Knoten
                        if (!isParked) {
                            lastSendTime = currentMillis;
                            isParked = true;
                        } else if (currentMillis - lastSendTime >= 1800000) {
                            sendData();
                            lastSendTime = currentMillis;
                        }
                    } else if (currentMillis - lastSendTime >= 2000 || isParked) {
                        sendData();
                        lastSendTime = currentMillis;
                        isParked = false;
                    }
                }
            }
        }
        lastGPSCheckTime = currentMillis; // Aktualisieren des Zeitpunkts der letzten GPS-Datenverarbeitung
    }
}



void manageWaterAndHeating() {
    if (pressureCmFiltered <= MinDruck || !Wasserversorgung) {
        Boilerheizung = false;
        Wasserversorgung = false;
        if (relayStates[5]) { // Überprüfen Sie, ob das Relais bereits ausgeschaltet ist
            sr.set(5, LOW);
            client.publish("/relay/6", "false");
            relayStates[5] = false;
        }
        setHeatingPower(0);
    } else {
        sr.set(5, HIGH);
        client.publish("/relay/6", "true");
        if (Boilerheizung) {
            static unsigned long heatingStartTime = millis();
                client.publish("/Boilerheizung", "true");
                client.publish("/ssr", "true");
                heatingStartTime = millis();
            if (millis() - heatingStartTime > 10000) {
                setHeatingPower(ssrPower);
                
            }
        } else {
            
        }
    }
}








void berechneUndSendeDruck() {
    static unsigned long lastExecutionTime = 0; // Zeitpunkt der letzten Ausführung
    unsigned long currentMillis = millis();

    // Überprüfen, ob seit der letzten Ausführung 2 Sekunden vergangen sind
    if (currentMillis - lastExecutionTime >= 2000) {
        int sensorValue = analogRead(SENSOR_PIN);
        float voltage = sensorValue * (3.3 / 4095.0);  // Umwandlung des ADC-Wertes in eine Spannung

        // Direkte Umrechnung der Spannung in Druck
        float pressure_kPa = mapf(voltage, 0.0, 3.3, MIN_PRESSURE, MAX_PRESSURE);

        // Umrechnung von kPa in cm Wassersäule
        float pressure_cm = pressure_kPa / 9.81;
        pressureCmFiltered = alpha * pressure_cm + (1.0 - alpha) * pressureCmFiltered;

        // Umrechnung von cm in Liter
        float pressure_L = pressureCmFiltered * 3.2;

        if (isParked && abs(pressureCmFiltered - lastSentPressureCm) > 1.0 && currentMillis - lastPressureSentTime > 5000) {
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

        lastExecutionTime = currentMillis; // Aktualisieren des Zeitpunkts der letzten Ausführung
    }
}




void sendData() {

    kalmanGain = 0.4;
    double filteredLatitude = previousLatitude + kalmanGain * (GPS.latitudeDegrees - previousLatitude);
    double filteredLongitude = previousLongitude + kalmanGain * (GPS.longitudeDegrees - previousLongitude);
    isOutlier = abs(filteredLatitude - GPS.latitudeDegrees) > outlierThreshold || abs(filteredLongitude - GPS.longitudeDegrees) > outlierThreshold;
    previousLatitude = filteredLatitude;
    previousLongitude = filteredLongitude;
    if (!isOutlier) {
        char latMsg[50], lonMsg[50], speedMsg[50], altMsg[50];
        snprintf(latMsg, 50, "%f", filteredLatitude);
        snprintf(lonMsg, 50, "%f", filteredLongitude);
        snprintf(speedMsg, 50, "%f", GPS.speed);  // Assuming speed is in knots
        snprintf(altMsg, 50, "%f", GPS.altitude);  // Assuming altitude is in meters
        client.publish("latitude", latMsg);
        client.publish("longitude", lonMsg);
        client.publish("speed", speedMsg);
        client.publish("altitude", altMsg);

        // Debug-Ausgaben für die gesendeten Daten
        
    } else {
    }
}


void setup_wifi() {
    
    WiFi.begin(ssid, password);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
        delay(500); 
    }
    if (WiFi.status() == WL_CONNECTED) {
    } else {
        Serial.println("Failed to connect to WiFi.");
    }
}


void handleRelays(int index, bool state) {
    sr.set(index, state ? HIGH : LOW);
}

void callback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';  // Füge ein Nullzeichen am Ende der Payload hinzu, um sie in einen gültigen C-String zu konvertieren
    String strPayload = String((char*)payload);  // Konvertiere Payload in einen Arduino String

    
    if (strncmp(topic, "/relay/", 7) == 0) {
        int relayIndex = topic[7] - '1';
        if (relayIndex >= 0 && relayIndex < 6) {
            relayStates[relayIndex] = strcmp((char*)payload, "true") == 0;
            handleRelays(relayIndex, (char*)payload);
            return;
        }
    }

    if (String(topic) == "/Inverter") {
        if (strPayload == "true") {
            sr.set(1, HIGH);  // Setze das 5. Relais (0-indexed) auf HIGH
            client.publish("/relay/2", "true");
            relayStates[1] = true;
        } else if (strPayload == "false") {
            sr.set(1, LOW);  // Setze das 5. Relais (0-indexed) auf LOW
            client.publish("/relay/2", "false");
            relayStates[1] = false;
        }
                Serial.println("Setting Inverter relay");

    }



    if (String(topic) == "/Kuehlschrankgross") {
        if (strPayload == "true") {
            sr.set(2, HIGH);  // Setze das 5. Relais (0-indexed) auf HIGH
            client.publish("/relay/3", "true");
            relayStates[2] = true;
        } else if (strPayload == "false") {
            sr.set(2, LOW);  // Setze das 5. Relais (0-indexed) auf LOW
            client.publish("/relay/3", "false");
            relayStates[2] = false;
        }
                Serial.println("Setting Kuehlschrankgross relay");

    }

    if (String(topic) == "/Kuehlschrankklein") {
        if (strPayload == "true") {
            sr.set(3, HIGH);  // Setze das 5. Relais (0-indexed) auf HIGH
            client.publish("/relay/4", "true");
            relayStates[3] = true;
        } else if (strPayload == "false") {
            sr.set(3, LOW);  // Setze das 5. Relais (0-indexed) auf LOW
            client.publish("/relay/4", "false");
            relayStates[3] = false;
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
            sr.set(5, HIGH);
            client.publish("/relay/6", "true");
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
            client.publish("/ssr", "true");
            setHeatingPower(ssrPower);

        } else if (strPayload == "false") {
            Boilerheizung = false;
            client.publish("/ssr", "false");
            setHeatingPower(0);
        }
    }

  

    const char* topicStr = (char*)topic;
    if (strcmp(topicStr, "/ssrPower") == 0) {
        ssrPower = atoi((char*)payload);
        Serial.println(ssrPower);
    } else if (strcmp(topicStr, "/Wasserversorgung") == 0) {
        Wasserversorgung = strcmp((char*)payload, "true") == 0;
    } else if (strcmp(topicStr, "/Boilerheizung") == 0) {
        Boilerheizung = strcmp((char*)payload, "true") == 0;
    } else if (strcmp(topicStr, "/UVCLicht") == 0) {
        UVCLicht = strcmp((char*)payload, "true") == 0;
    } else if (strcmp(topicStr, "MinDruck") == 0) {
        MinDruck = atof((char*)payload);
    }   else if (strcmp(topicStr, "/Kuehlschrankgross") == 0) {
        Kuehlschrankgross = strcmp((char*)payload, "true") == 0;
    }   else if (strcmp(topicStr, "/Kuehlschrankklein") == 0) {
        Kuehlschrankklein = strcmp((char*)payload, "true") == 0;
    }   else if (strcmp(topicStr, "/Inverter") == 0) {
        Inverter = strcmp((char*)payload, "true") == 0;
    }
}

void handleRoot() {
    String html = "<html><body>";
    html += "<h1>Einstellungen</h1>";

    html += "<h2>Relais Zustände</h2>";
    for (int i = 0; i < 6; i++) {
        html += "Relais " + String(i + 1) + ": " + (relayStates[i] ? "true" : "false") + "<br>";
    }

    html += "<h2>SSR Zustand</h2>";
    html += "SSR: " + String(ssrState ? "true" : "false") + "<br>";
    html += "SSR Power: " + String(ssrPower) + "%<br>";

    html += "<h2>Wasservolumen</h2>";
    html += "Aktuelles Wasservolumen: " + String(pressureCmFiltered * 3.55) + " Liter<br>";

    html += "</body></html>";
    server.send(200, "text/html", html);
}

boolean reconnect() {
    unsigned long lastAttemptTime = millis();
    while (!client.connected()) {
        if (millis() - lastAttemptTime >= 5000) { 
            if (client.connect("ESP32Client")) {
                Serial.println("connected");
                // Abonnieren von relay Topics
                

                // Abonnieren von weiteren Topics
                client.publish("/ssr", "false");
                client.subscribe("/ssr");
                client.subscribe("/ssrPower");
                client.subscribe("/MinDruck");
                client.subscribe("/Wasserversorgung");
                client.subscribe("/Boilerheizung");
                client.subscribe("/UVCLicht");
                client.subscribe("/Kuehlschrankgross");
                client.subscribe("/Kuehlschrankklein");
                client.subscribe("/Inverter");
                client.subscribe("/heartbeat");

                return true;
            } else {
                lastAttemptTime = millis();
            }
        }
    }
    return false;
}
