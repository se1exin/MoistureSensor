// Wifi username and password
const char *WIFI_SSID = "your ssid";
const char *WIFI_PASSWORD = "your password";

// MQTT connection details
const char *MQTT_SERVER = "x.x.x.x";
const char *MQTT_USER;
const char *MQTT_PASSWORD;
const char *MQTT_CLIENT_ID = "esp8266_moisture_01";

#define MQTT_TOPIC_STATE "home/esp8266_moisture_01/state"
#define MQTT_TOPIC_STATUS "home/esp8266_moisture_01/status"
#define MQTT_TOPIC_MOISTURE "home/esp8266_moisture_01/moisture"
#define MQTT_TOPIC_HUMIDITY "home/esp8266_moisture_01/humidity"
#define MQTT_TOPIC_TEMPERATURE "home/esp8266_moisture_01/temperature"
#define MQTT_TOPIC_PRESSURE "home/esp8266_moisture_01/pressure"
