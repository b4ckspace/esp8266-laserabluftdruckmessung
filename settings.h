#define BUTTON_PIN D4
#define NUM_BMP280 7

#define I2C_DISPLAY 7

#define I2C_ADDR_BMP280 0x76
#define I2C_ADDR_MULTIPLEXER 0x70

#define TIME_SECOND_MS 1000

#define HOSTNAME "ESP-LaserPressure"

#define WIFI_SSID "---"
#define WIFI_PASSWORD "---"

#define OTA_PASSWORD "---"

#define MQTT_HOST "mqtt.core.bckspc.de"
#define MQTT_PORT 1883

#define MQTT_TOPIC_STATE "state/laser/sensor"
#define MQTT_TOPIC_LASER_OPERATION "project/laser/active"
#define MQTT_TOPIC_SENSOR_PRESSURE_BASE "project/laser/sensor/pressure"
#define MQTT_TOPIC_SENSOR_TEMPERATURE_BASE "project/laser/sensor/temperature"

#define SENSOR_POLL_INTERVAL_INACTIVE_MS (5 * TIME_SECOND_MS)
#define SENSOR_POLL_INTERVAL_ACTIVE_MS  TIME_SECOND_MS

typedef struct {
  Adafruit_BMP280 sensor;
  bool connected;
  uint8_t consecutiveErrors;
  float temperature;
  float pressure;
} PressureSensor;
