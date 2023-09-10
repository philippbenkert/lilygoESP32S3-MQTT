***** GROSSER DANK GEHT AN DIE ENTWICKLER VON CHATGPT! DER CODE IST IN 2 STUNDEN ENTSTANDEN, DIE MÖGLICHKEITEN IDEEN UMZUSETZEN SIND BEMERKENSWERT. DANK DER PLUGINS IST DIE BEGRENTZUNG DES WISSENSSTANDES AUF SEPTEMBER 2021 QUASI AUFGEHOBEN. ZUM DAMALIGEN ZEITPUNKT GAB ES DEN ESP32S3 ÜBERHAUPT NOCH NICHT UND DEMENTSPRECHEND KONNTEN KEINE DEMENTSPRECHENDEN CODES INS TRAININGSPROGRAMM VON CHATGPT 4 EINFLIESSEN. *****

# README: ESP32 Water Management System

## Overview

This code is designed for an ESP32-based system that manages various functionalities related to water management, GPS tracking, and device control. The system integrates with MQTT for communication, uses a GPS module for location tracking, and manages multiple relays and sensors.

## Features

1. **WiFi Connectivity**: Connects to a specified WiFi network.
2. **MQTT Integration**: Communicates with an MQTT broker to send and receive data.
3. **GPS Tracking**: Uses an Adafruit GPS module to track the device's location.
4. **Relay Control**: Manages multiple relays to control devices.
5. **Temperature Sensing**: Integrates with a DS18B20 temperature sensor.
6. **Pressure Sensing**: Measures water pressure and calculates water volume.
7. **Over-the-Air (OTA) Updates**: Allows for remote firmware updates.
8. **Web Server**: Hosts a web server for device settings and status.

## Dependencies

- `WiFi.h`: For WiFi connectivity.
- `PubSubClient.h`: For MQTT communication.
- `ShiftRegister74HC595.h`: To manage shift registers.
- `Adafruit_GPS.h`: For GPS functionality.
- `SoftwareSerial.h`: Software-based serial communication.
- `ESPmDNS.h`: For mDNS functionality.
- `WiFiUdp.h`: UDP protocol for OTA.
- `ArduinoOTA.h`: For OTA updates.
- `WebServer.h`: To host a web server.
- `Preferences.h`: To store and retrieve preferences.
- `OneWire.h`: For DS18B20 temperature sensor communication.
- `DallasTemperature.h`: Library for the DS18B20 sensor.
- `U8g2lib.h`: For the U-blox M8N GPS receiver.

## Setup

1. **WiFi Credentials**: Update the `ssid` and `password` variables with your WiFi credentials.
2. **MQTT Server**: Update the `mqtt_server` variable with the IP address of your MQTT broker.
3. **GPS Configuration**: The GPS module is set to communicate at a baud rate of 9600 and is configured to output RMC and GGA NMEA sentences at a 1Hz update rate.
4. **Web Server**: The embedded web server runs on port 80 and provides endpoints for viewing and updating settings.

## Usage

1. **Powering On**: Upon startup, the system will attempt to connect to the specified WiFi network and the MQTT broker.
2. **GPS Data**: The system reads GPS data and filters it using a Kalman filter. If the device is stationary (speed < 2 knots), it will send the GPS data to the MQTT broker.
3. **Pressure Sensing**: The system reads the pressure sensor value, converts it to a voltage, and then calculates the water pressure in kPa and cm of water column. If the device is stationary, it sends the water volume (in liters) to the MQTT broker.
4. **Temperature Sensing**: The system reads the temperature from the DS18B20 sensor and sends it to the MQTT broker every 20 seconds.
5. **Relay Control**: The system can control up to 6 relays. The state of each relay can be changed via MQTT messages.
6. **Web Interface**: Access the device's web interface by navigating to `http://esp32.local/` in a web browser. From here, you can view the current relay states, SSR state, and water volume. You can also update the WiFi credentials and other settings.

## MQTT Topics

- **Relay Control**: `/relay/[1-6]` - Control individual relays. Send "true" to turn on and "false" to turn off.
- **SSR Control**: `/ssr` and `/ssrPower` - Control the SSR state and power level.
- **Water Supply**: `/Wasserversorgung` - Control the water supply. Send "true" to turn on and "false" to turn off.
- **Boiler Heating**: `/Boilerheizung` - Control the boiler heating. Send "true" to turn on and "false" to turn off.
- **UVC Light**: `/UVCLicht` - Control the UVC light. Send "true" to turn on and "false" to turn off.
- **Refrigerators**: `/Kuehlschrankgross` and `/Kuehlschrankklein` - Control the large and small refrigerators, respectively.
- **Inverter**: `/Inverter` - Control the inverter.

## Web Interface

The web interface provides a form to update the WiFi credentials, minimum pressure threshold, and relay states. After updating the settings, they are saved to the ESP32's preferences and used on subsequent startups.

## Troubleshooting

1. **WiFi Connection Issues**: Ensure the `ssid` and `password` are correctly set. Check the serial output for connection status.
2. **MQTT Connection Issues**: Ensure the

 MQTT broker is running and the IP address is correctly set in the `mqtt_server` variable.
3. **GPS Data**: If no GPS data is received, check the GPS module's wiring and ensure it has a clear view of the sky.
4. **OTA Updates**: Ensure the device is connected to the same network as the computer from which you are sending the OTA update.

## Contributing

If you wish to contribute to this project, please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Acknowledgments

Thanks to the open-source community for providing the libraries and tools that made this project possible.
