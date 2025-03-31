/***************************************************
   Includes & Global Declarations
 ***************************************************/
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_BME280.h>

#include "secrets.h"

// Instantiate sensor objects
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_BME280   bme;
WiFiClient        wifiClient;
PubSubClient      mqttClient(wifiClient);

// Flags to indicate sensor availability
bool vemlAvailable = false;
bool bmeAvailable  = false;

// MQTT client ID (should be unique per device)
const char* mqttClientId = "ESP32_SensorClient";

// Interval for publishing data (milliseconds)
static const unsigned long PUBLISH_INTERVAL_MS = 1000;
unsigned long lastPublishTime = 0;

/***************************************************
   AC Voltage Sensor (AMPT101B) Definitions
 ***************************************************/
#define AC_SENSOR_PIN            34     // ESP32 ADC pin
#define ADC_MAX_VALUE            4095   // 12-bit ADC for ESP32
#define ADC_REF_VOLTAGE          3.3    // ESP32 reference voltage
#define AMPT101B_VOLTAGE_FACTOR  185.0  // Calibration factor for AMPT101B

/***************************************************
   VEML7700 Definitions
 ***************************************************/


/***************************************************
   BME280 Definitions
  ***************************************************/
#define TEMPERATURE_OFFSET -2.0
#define PRESSURE_OFFSET    0.0
#define HUMIDITY_OFFSET    13.0

/***************************************************
   Function Prototypes
 ***************************************************/
void setupWiFi();
void setupMQTT();
void reconnectMQTT();
void setupSensors();
void publishAllSensors();  // Publishes separate JSON messages for each sensor reading
void publishSensorReading(const String &sensorType, float value, const String &unit);
float readVoltageAC();
float readLux();
float readTemperature();
float readHumidity();
float readPressure();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- Starting Up ---");

  // Initialize I2C
  Wire.begin();

  // Initialize sensors (sets flags if sensors are found)
  setupSensors();

  // Connect Wi-Fi
  setupWiFi();

  // Set up MQTT client
  mqttClient.setKeepAlive(60);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  setupMQTT();
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT(); // Non-blocking, attempts to reconnect
  } else {
    mqttClient.loop(); // MUST run often
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishTime >= PUBLISH_INTERVAL_MS) {
    lastPublishTime = currentMillis;
    publishAllSensors();
  }
}

/***************************************************
   Setup Wi-Fi
 ***************************************************/
void setupWiFi() {
  Serial.print("Connecting to Wi-Fi: ");
  Serial.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWi-Fi connected. IP address: ");
  Serial.println(WiFi.localIP());
}

/***************************************************
   Setup MQTT
 ***************************************************/
void setupMQTT() {
  reconnectMQTT();
}

/***************************************************
   Reconnect to MQTT (if disconnected)
 ***************************************************/
void reconnectMQTT() {
  static unsigned long lastReconnectAttempt = 0;
  unsigned long now = millis();
  if (now - lastReconnectAttempt > 5000) { // Try every 5 sec
    lastReconnectAttempt = now;
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("MQTT reconnected successfully");
    } else {
      Serial.print("MQTT reconnection failed, rc=");
      Serial.println(mqttClient.state());
    }
  }
}


/***************************************************
   Setup Sensors
 ***************************************************/
void setupSensors() {
  // VEML7700
  if (veml.begin()) {
    vemlAvailable = true;
    Serial.println("VEML7700 found.");
  } else {
    vemlAvailable = false;
    Serial.println("VEML7700 NOT found. Continuing without it.");
  }

  // BME280 (addresses 0x76)
  if (bme.begin(0x76)) {
    bmeAvailable = true;
    Serial.println("BME280 found.");
  } else {
    bmeAvailable = false;
    Serial.println("BME280 NOT found. Continuing without it.");
  }

  // The AMPT101B sensor is read via analog pin, so no special setup is needed here.
}

/***************************************************
   Publish All Sensors
   (One JSON message per sensor reading)
 ***************************************************/
void publishAllSensors() {
  // 1) AC Voltage
  float acVoltage = readVoltageAC();
  publishSensorReading("voltage", acVoltage, "V");

  // 2) Lux (only if VEML is available)
  if (vemlAvailable) {
    float lux = readLux();
    publishSensorReading("light", lux, "lux");
  }

  // 3) Temperature, Humidity, Pressure (only if BME is available)
  if (bmeAvailable) {
    float temperature = readTemperature();  // °C
    float humidity    = readHumidity();     // %
    float pressure    = readPressure();     // hPa

    publishSensorReading("temperature", temperature, "C");
    publishSensorReading("humidity",    humidity,    "%");
    publishSensorReading("pressure",    pressure,    "hPa");
  }
}

/***************************************************
   Publish a Single Sensor Reading
   Example JSON:
   {
     "sensor_type": "temperature",
     "value": 25.4,
     "unit": "C"
   }
 ***************************************************/
void publishSensorReading(const String &sensorType, float value, const String &unit) {
  String payload = "{";
  payload += "\"sensor_type\":\"" + sensorType + "\",";
  payload += "\"value\":" + String(value, 2) + ",";
  payload += "\"unit\":\"" + unit + "\"";
  payload += "}";

  // Publish to MQTT
  if (mqttClient.publish(MQTT_TOPIC, payload.c_str())) {
    Serial.print("Published [");
    Serial.print(MQTT_TOPIC);
    Serial.print("]: ");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish sensor reading.");
  }
}

/***************************************************
   AMPT101B Voltage Sensor Reading
   This module typically outputs a DC level proportional
   to the RMS AC voltage.
 ***************************************************/
float readVoltageAC() {
  int rawVal = analogRead(AC_SENSOR_PIN);

  // Convert ADC reading to the corresponding sensor output voltage
  float sensorVoltage = (float)rawVal / (float)ADC_MAX_VALUE * ADC_REF_VOLTAGE;

  // Scale up to approximate actual AC RMS voltage
  // Adjust AMPT101B_VOLTAGE_FACTOR to calibrate your specific sensor
  float lineVoltage = sensorVoltage * AMPT101B_VOLTAGE_FACTOR;

  return lineVoltage;
}

/***************************************************
   VEML7700 - readLux()
 ***************************************************/
float readLux() {
  // We only call this if vemlAvailable is true.
  return veml.readLux();
}

/***************************************************
   BME280 - read temperature, humidity, pressure
 ***************************************************/
float readTemperature() {
  return bme.readTemperature() + TEMPERATURE_OFFSET; // °C
}

float readHumidity() {
  return bme.readHumidity() + HUMIDITY_OFFSET; // %
}

float readPressure() {
  return bme.readPressure() / 100.0F + PRESSURE_OFFSET; // hPa
}
