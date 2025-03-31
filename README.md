# iamis-esp

iamis-esp is an IoT sensor node project based on the ESP32 microcontroller. It collects environmental data from multiple sensors and publishes real-time sensor readings via MQTT in JSON format. This project is ideal for integration into IoT dashboards, remote monitoring systems, or data logging applications.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Setup and Installation](#setup-and-installation)
- [Configuration](#configuration)
- [Wiring Diagram](#wiring-diagram)
- [Code Structure](#code-structure)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Contact](#contact)

---

## Overview

iamis-esp is designed to monitor various environmental parameters and AC voltage. It uses:
- **AMPT101B AC Voltage Sensor** to measure AC voltage via an analog input.
- **VEML7700 Ambient Light Sensor** to measure ambient light levels.
- **BME280 Sensor** to obtain temperature, humidity, and pressure readings.

The sensor node connects to a Wi-Fi network and publishes each sensor reading as a JSON message over MQTT at a configurable interval (default every 1000 milliseconds).

---

## Features

- **Multi-Sensor Integration:**  
  Reads data from:
  - **AC Voltage Sensor (AMPT101B):** Provides AC voltage measurements.
  - **Ambient Light Sensor (VEML7700):** Measures lux levels.
  - **Environmental Sensor (BME280):** Measures temperature, humidity, and pressure.

- **MQTT Communication:**  
  Publishes sensor readings as JSON messages to an MQTT broker.

- **Wi-Fi Connectivity:**  
  Connects to a specified Wi-Fi network for remote data transmission.

- **Robust Reconnection Logic:**  
  Implements non-blocking MQTT reconnection to ensure continuous data publishing.

- **Configurable Publish Interval:**  
  Sensor data is published every second (by default), with an option to adjust the interval.

---

## Hardware Requirements

- **ESP32 Development Board:** (e.g., ESP32 DevKitC)
- **AMPT101B AC Voltage Sensor**
- **VEML7700 Ambient Light Sensor Module**
- **BME280 Sensor Module** (for temperature, humidity, and pressure)
- **Breadboard and Connecting Wires**
- **Appropriate Power Supply** for the ESP32 and sensors

---

## Software Requirements

- **Arduino IDE** or **PlatformIO** for code compilation and uploading.
- **ESP32 Board Support Package** installed in your development environment.
- **Required Libraries:**
  - **Wire** (for I2C communication)
  - **WiFi** (for ESP32 connectivity)
  - **PubSubClient** (for MQTT communication)
  - **Adafruit VEML7700 Library**
  - **Adafruit BME280 Library**

Install these libraries via the PlatformIO/Arduino Library Manager if they are not already installed.

---

## Setup and Installation

1. **Clone or Download the Repository:**

   ```bash
   git clone https://github.com/cirelote/iamis-esp.git
   ```
   
2. **Open the Project:**
   Open the main code file in Arduino IDE or PlatformIO.

3. **Install Required Libraries:**
   Use the Platform/Arduino Library Manager to install:
   - PubSubClient
   - Adafruit VEML7700
   - Adafruit BME280

4. **Select the Correct Board and Port:**
   In your IDE, choose your ESP32 board and the appropriate serial port.

---

## Configuration

Create a file named `secrets.h` in the project folder (alongside your main source file) to store your Wi-Fi and MQTT credentials. Below is an example template:

```cpp
#ifndef SECRETS_H
#define SECRETS_H

// Wi-Fi credentials
#define WIFI_SSID       "YourWiFiSSID"
#define WIFI_PASSWORD   "YourWiFiPassword"

// MQTT Broker settings
#define MQTT_HOST       "your.mqtt.broker.address"
#define MQTT_PORT       1883
#define MQTT_TOPIC      "sensor/data"

#endif // SECRETS_H
```

Replace the placeholder values with your actual credentials and MQTT broker details.

---

## Wiring Diagram

A basic wiring guide:

- **AMPT101B AC Voltage Sensor:**
  - Connect its output to **ADC Pin 34** on the ESP32.

- **VEML7700 Ambient Light Sensor:**
  - Connect via I2C:
    - SDA to ESP32 SDA (typically GPIO 21)
    - SCL to ESP32 SCL (typically GPIO 22)

- **BME280 Sensor:**
  - Connect via I2C:
    - SDA to ESP32 SDA (GPIO 21)
    - SCL to ESP32 SCL (GPIO 22)
  - Default I2C address is `0x76` (adjust in code if different).

Ensure that all sensors share a common ground with the ESP32.

---

## Code Structure

- **Includes & Global Declarations:**  
  Initializes libraries and declares global sensor objects and configuration parameters.

- **Wi-Fi and MQTT Setup:**  
  - `setupWiFi()`: Connects to the Wi-Fi network.
  - `setupMQTT()` and `reconnectMQTT()`: Establish and maintain the MQTT connection.

- **Sensor Setup and Reading:**  
  - `setupSensors()`: Initializes the VEML7700 and BME280 sensors and sets availability flags.
  - `readVoltageAC()`: Reads AC voltage from the AMPT101B sensor.
  - `readLux()`, `readTemperature()`, `readHumidity()`, `readPressure()`: Retrieve sensor readings.

- **Publishing Data:**  
  - `publishSensorReading()`: Formats and publishes a sensor reading as a JSON message.
  - `publishAllSensors()`: Collects data from all sensors and publishes each reading individually.

- **Main Loop:**  
  Maintains MQTT connectivity and publishes sensor data at a set interval (every 1000 ms by default).

---

## Usage

1. **Upload the Code:**  
   Connect your ESP32 board, select the correct board and port, then upload the sketch via Arduino IDE or PlatformIO.

2. **Monitor Serial Output:**  
   Open the Serial Monitor (set to 115200 baud) to check Wi-Fi connection status, MQTT reconnection attempts, and sensor readings.

3. **View Sensor Data via MQTT:**  
   Use an MQTT client (e.g., MQTT Explorer, Node-RED) to subscribe to the topic defined in `MQTT_TOPIC` and verify that sensor data is published in JSON format.

---

## Troubleshooting

- **Wi-Fi Connection Issues:**
  - Verify that the Wi-Fi credentials in `secrets.h` are correct.
  - Ensure the ESP32 is within range of the Wi-Fi network.

- **MQTT Connection Problems:**
  - Confirm that the MQTT broker is running and accessible.
  - Check that the MQTT_HOST and MQTT_PORT values in `secrets.h` are accurate.
  - Review the Serial Monitor for MQTT connection error codes.

- **Sensor Data Issues:**
  - Ensure sensors are properly wired and powered.
  - Check the Serial Monitor to see if any sensor initialization errors are reported.
  - Adjust calibration factors (e.g., `AMPT101B_VOLTAGE_FACTOR`, temperature/humidity offsets) as necessary.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Contact

For questions, support, or contributions, please contact:

- **Project Maintainer:** [Bohdan Lutsenko](mailto:bohdan.lutsen.co@gmail.com)
- **GitHub:** [cirelote](https://github.com/cirelote)
