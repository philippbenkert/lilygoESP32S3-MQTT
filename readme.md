***** GROSSER DANK GEHT AN DIE ENTWICKLER VON CHATGPT! DER CODE IST IN 2 STUNDEN ENTSTANDEN, DIE MÖGLICHKEITEN IDEEN UMZUSETZEN SIND BEMERKENSWERT. DANK DER PLUGINS IST DIE BEGRENTZUNG DES WISSENSSTANDES AUF SEPTEMBER 2021 QUASI AUFGEHOBEN. ZUM DAMALIGEN ZEITPUNKT GAB ES DEN ESP32S3 ÜBERHAUPT NOCH NICHT UND DEMENTSPRECHEND KONNTEN KEINE DEMENTSPRECHENDEN CODES INS TRAININGSPROGRAMM VON CHATGPT 4 EINFLIESSEN. *****

README.md
Projektbeschreibung
Dieses Projekt ist ein GPS-Tracker, der auf einem ESP32S3 basiert und Daten an einen MQTT-Broker sendet. Es verwendet einen Kalman-Filter, um die GPS-Daten zu glätten, und erkennt Ausreißer in den Daten. Zusätzlich liest es Daten von einem 4-20mA Drucksensor und sendet diese ebenfalls an den MQTT-Broker. Es unterstützt auch OTA-Updates und die Steuerung von Relais und einem Solid State Relay (SSR) über MQTT. Die GPS-Daten werden in Abhängigkeit davon, ob das Fahrzeug geparkt ist oder sich bewegt, gesendet.

README: ESP32S3 Camper-Hausleitsystem
Beschreibung:
Das ESP32S3-System ist für den Einsatz in einem Camper konzipiert und ermöglicht die Integration in ein Hausleitsystem. Es verwendet MQTT zur Kommunikation und verfügt über verschiedene Sensoren und Aktoren.

Funktionen:
WiFi-Verbindung: Das System stellt eine Verbindung zu einem vordefinierten WiFi-Netzwerk her.
MQTT-Kommunikation: Es kommuniziert mit einem MQTT-Broker, um Daten zu senden und Befehle zu empfangen.
GPS-Datenverarbeitung: Mit einem GPS-Modul werden Standortdaten erfasst und verarbeitet.
Tankfüllstandsmessung: Ein 4-20mA 0-10kPa Drucksensor misst den Druck im Wassertank und sendet die Daten an den MQTT-Broker. Dies wird verwendet, um den Füllstand des Tanks zu bestimmen.
Relaissteuerung: Es gibt mehrere Relais, die über MQTT-Befehle gesteuert werden können.
Heizungssteuerung (Boilerheizung): Ein SSR (Solid State Relay) steuert die Heizung mit PWM (Pulsweitenmodulation). Die Heizung kann über MQTT ein- und ausgeschaltet werden.
Wasserversorgung: Eine Wasserpumpe wird über ein Relais gesteuert. Die Pumpe kann über MQTT ein- und ausgeschaltet werden.
OTA-Updates: Das System unterstützt Over-the-Air-Updates.
Sicherheitsüberprüfungen: Es gibt Sicherheitsüberprüfungen, um sicherzustellen, dass bestimmte Bedingungen erfüllt sind, bevor Aktionen ausgeführt werden (z.B. Heizung einschalten, wenn der Druck zu niedrig ist).
MQTT-Variablen und Zustände:
/relay1 bis /relay6: Steuert die Relais. Mögliche Zustände: "on", "off".
/ssrPower: Steuert die Heizleistung des SSR. Akzeptiert Werte zwischen 0 (aus) und 100 (maximale Leistung).
/ssr: Steuert den SSR direkt. Mögliche Zustände: "on", "off".
/Wasserversorgung: Steuert die Wasserpumpe. Mögliche Zustände: "on", "off".
/Boilerheizung: Steuert die Boilerheizung. Mögliche Zustände: "on", "off".
/UVCLicht: Steuert das UVC-Licht. Mögliche Zustände: "on", "off".
/MinDruck: Setzt den minimalen Druckwert. Akzeptiert Fließkommawerte.
/TankLimitDruck: Informiert über den Tankdruckgrenzwert. Akzeptiert Fließkommawerte.
pressure: Sendet den aktuellen Druckwert, der den Tankfüllstand repräsentiert. Akzeptiert Fließkommawerte.
latitude: Sendet den aktuellen Breitengrad. Akzeptiert Fließkommawerte.
longitude: Sendet den aktuellen Längengrad. Akzeptiert Fließkommawerte.
speed: Sendet die aktuelle Geschwindigkeit. Akzeptiert Fließkommawerte.
altitude: Sendet die aktuelle Höhe. Akzeptiert Fließkommawerte.
Schaltplan:
ESP32S3: Das Hauptboard, das alle Komponenten steuert.
GPS-Modul: An die Pins 11 (RX) und 13 (TX) angeschlossen.
4-20mA 0-10kPa Drucksensor: An Pin 10 (SENSOR_PIN) angeschlossen. Dieser Sensor misst den Druck im Wassertank und wird verwendet, um den Füllstand des Tanks zu bestimmen.
Shift Register (74HC595): Verwendet die Pins 7 (SER_Pin), 5 (RCLK_Pin) und 6 (SRCLK_Pin).
SSR (Solid State Relay): An Pin 12 (ssrPin) angeschlossen und steuert die Heizung.
Relais: Sechs Relais, die über das Shift Register gesteuert werden. Eines dieser Relais steuert die Wasserpumpe.
WiFi: Verbindet sich mit dem vordefinierten Netzwerk "SSID" mit dem Passwort "PASSWORD".
MQTT: Kommuniziert mit dem MQTT-Broker unter der IP "192.168.0.1" und Port 1024.
Hinweis:
Bitte stellen Sie sicher, dass Sie alle Komponenten gemäß dem Schaltplan anschließen und die notwendigen Bibliotheken installieren, bevor Sie den Code auf Ihren ESP32S3 hochladen.


Wenn Sie immer noch Probleme haben, können Sie sich an die Community wenden oder ein Problem in diesem Repository eröffnen.
