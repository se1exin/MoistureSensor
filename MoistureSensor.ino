#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "constants.h"

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

WiFiClient espClient;
PubSubClient mqttClient(espClient);

int moisture = 0;
int sleepSeconds = 300;
int sleepNanos = sleepSeconds * 1000000;

void setup() {
  Wire.begin(4, 5);
  Serial.begin(115200);
  
  while (! Serial);

  setupWifi();
  mqttClient.setServer(MQTT_SERVER, 1883);
  if (!mqttClient.connected()) {
    mqttReconnect();
  }

  if (!bme.begin()) {
    while (1) delay(10);
  }
}

void loop() {
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  delay(500);

  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  moisture = analogRead(A0);

  mqttPublish(MQTT_TOPIC_MOISTURE, moisture);
  mqttPublish(MQTT_TOPIC_HUMIDITY, String(humidity_event.relative_humidity, 2));
  mqttPublish(MQTT_TOPIC_TEMPERATURE, String(temp_event.temperature, 2));
  mqttPublish(MQTT_TOPIC_PRESSURE, String(pressure_event.pressure, 2));
  
  delay(1000);

  ESP.deepSleep(sleepNanos);
}



/**
 * WIFI / MQTT HELPERS
 */
void setupWifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATUS, 1, true, "disconnected", false)) {
      Serial.println("connected");

      // Once connected, publish an announcement...
      mqttClient.publish(MQTT_TOPIC_STATUS, "connected", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttPublish(char *topic, int payload) {
  // Serial.print(topic);
  // Serial.print(": ");
  // Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}

void mqttPublish(char *topic, String payload) {
  // Serial.print(topic);
  // Serial.print(": ");
  // Serial.println(payload);

  mqttClient.publish(topic, payload.c_str(), true);
}
