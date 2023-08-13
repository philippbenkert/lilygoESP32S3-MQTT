***** GROSSER DANK GEHT AN DIE ENTWICKLER VON CHATGPT! DER CODE IST IN 2 STUNDEN ENTSTANDEN, DIE MÖGLICHKEITEN IDEEN UMZUSETZEN SIND BEMERKENSWERT. DANK DER PLUGINS IST DIE BEGRENTZUNG DES WISSENSSTANDES AUF SEPTEMBER 2021 QUASI AUFGEHOBEN. ZUM DAMALIGEN ZEITPUNKT GAB ES DEN ESP32S3 ÜBERHAUPT NOCH NICHT UND DEMENTSPRECHEND KONNTEN KEINE DEMENTSPRECHENDEN CODES INS TRAININGSPROGRAMM VON CHATGPT 4 EINFLIESSEN. *****

README.md
Projektbeschreibung
Dieses Projekt ist ein GPS-Tracker, der auf einem ESP32S3 basiert und Daten an einen MQTT-Broker sendet. Es verwendet einen Kalman-Filter, um die GPS-Daten zu glätten, und erkennt Ausreißer in den Daten. Zusätzlich liest es Daten von einem 4-20mA Drucksensor und sendet diese ebenfalls an den MQTT-Broker. Es unterstützt auch OTA-Updates und die Steuerung von Relais und einem Solid State Relay (SSR) über MQTT. Die GPS-Daten werden in Abhängigkeit davon, ob das Fahrzeug geparkt ist oder sich bewegt, gesendet.

Drucksensor
Der Code enthält auch eine Implementierung für die Verarbeitung von Daten von einem 4-20mA Drucksensor. Der Sensor wird über einen Shunt-Widerstand angeschlossen, um den Strom in eine Spannung umzuwandeln, die vom ESP32S3 gelesen werden kann. Der Druck wird dann basierend auf dem gelesenen Strom berechnet.

## Abhängigkeiten

Dieses Projekt verwendet die folgenden Bibliotheken:

- WiFi.h
- PubSubClient.h
- ShiftRegister74HC595.h
- TinyGPS++.h
- SoftwareSerial.h
- ESPmDNS.h
- WiFiUdp.h
- ArduinoOTA.h
- Update.h
- ESP32HTTPUpdateServer.h
- WebServer.h

Bitte stellen Sie sicher, dass Sie diese Bibliotheken installiert haben, bevor Sie versuchen, den Code zu kompilieren.

## Einrichtung

Um dieses Projekt zu verwenden, müssen Sie die folgenden Schritte ausführen:

1. Laden Sie den Code auf Ihren ESP32.
2. Ändern Sie die `ssid` und `password` Variablen auf die Anmeldedaten Ihres WiFi-Netzwerks.
3. Ändern Sie die `mqtt_server` Variable auf die Adresse Ihres MQTT-Brokers.
4. Verbinden Sie Ihren GPS-Sensor und Ihren zusätzlichen Sensor mit den entsprechenden Pins auf Ihrem ESP32.
5. Verbinden Sie Ihre Relais und das SSR mit den entsprechenden Pins auf Ihrem ESP32.

Die folgenden Pins werden in diesem Projekt verwendet:

- SensorPin: 10
- SER_Pin (Shift Register): 7
- RCLK_Pin (Shift Register): 5
- SRCLK_Pin (Shift Register): 6
- SSR_Pin: 12
- RX (GPS Sensor): 11
- TX (GPS Sensor): 13

Bitte stellen Sie sicher, dass Sie Ihre Geräte entsprechend anschließen.

Anschluss des 4-20mA Drucksensors
Der positive (+) Anschluss des Sensors sollte an die +10VDC Versorgung angeschlossen werden.
Der negative (-) oder Ausgangsanschluss des Sensors sollte an einen Ende des Shunt-Widerstands angeschlossen werden. Das andere Ende des Shunt-Widerstands sollte mit dem GND (Erdung) des ESP32S3 und der Stromversorgung verbunden sein.
Der Punkt zwischen dem Sensorausgang und dem Shunt-Widerstand (wo die Spannung abgegriffen wird) sollte mit dem SENSOR_PIN (Pin 10) des ESP32S3 verbunden werden.
Stellen Sie sicher, dass die erzeugte Spannung über dem Shunt-Widerstand innerhalb des zulässigen Eingangsspannungsbereichs des ESP32S3 liegt. Es könnte auch hilfreich sein, eine Schutzschaltung oder einen Spannungsteiler hinzuzufügen, um den Eingang des ESP32S3 zu schützen.

## Verwendung

Sobald der Code auf Ihrem ESP32 läuft und eine Verbindung zu Ihrem WiFi-Netzwerk und Ihrem MQTT-Broker hergestellt hat, beginnt er, GPS-Daten zu sammeln und zu senden. Es sendet auch Daten von dem angeschlossenen Sensor. Sie können die gesendeten Daten auf Ihrem MQTT-Broker überwachen.

Zusätzlich können Sie die Relais und das SSR über MQTT steuern, indem Sie Nachrichten an die entsprechenden Topics senden. Die GPS-Daten werden in Abhängigkeit davon, ob das Fahrzeug geparkt ist oder sich bewegt, gesendet. Wenn das Fahrzeug geparkt ist, werden die Daten alle 30 Minuten gesendet. Wenn das Fahrzeug sich bewegt, werden die Daten jede Sekunde gesendet.

## OTA-Updates

Dieses Projekt unterstützt OTA-Updates. Sie können neue Versionen des Codes auf Ihren ESP32 laden, ohne ihn physisch mit Ihrem Computer verbinden zu müssen. Um ein OTA-Update durchzuführen, navigieren Sie zu `http://esp32.local/update` in Ihrem Webbrowser.

## Fehlersuche

Wenn Sie Probleme mit diesem Projekt haben, überprüfen Sie bitte die folgenden Punkte:

- Stellen Sie sicher, dass Ihr ESP32 korrekt mit Ihrem WiFi-Netzwerk

verbunden ist.
- Stellen Sie sicher, dass Ihr MQTT-Broker läuft und erreichbar ist.
- Überprüfen Sie die Verbindungen zu Ihren Sensoren und Relais.
- Stellen Sie sicher, dass Sie die richtigen Bibliotheken installiert haben.

Wenn Sie immer noch Probleme haben, können Sie sich an die Community wenden oder ein Problem in diesem Repository eröffnen.
