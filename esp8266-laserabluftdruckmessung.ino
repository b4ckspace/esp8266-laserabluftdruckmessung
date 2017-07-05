#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SimpleTimer.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Bounce2.h>
#include "settings.h"
#include "PressureSensor.h"

SimpleTimer timer;
PubSubClient mqttClient;
WiFiClient wifiClient;

Encoder encoder(D6, D5);
LiquidCrystal_I2C lcd(0x27, 16, 2);

PressureSensor* sensors[NUM_BMP280];

Bounce debouncer = Bounce();

uint8_t selectedPressureSensor = 6;
bool blink = false;
bool showTemperature = false;
bool isLaserActive = false;
int updateSensorValuesTimerId = -1;
int laserInactiveTimeoutTimerId = -1;

char formatBuffer[128] = {0};
char valueBuffer[42] = {0};

void i2c_select(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(I2C_ADDR_MULTIPLEXER);
  Wire.write(1 << i);
  Wire.endTransmission();

  delay(10);
}

void setup() {
  delay(500);
  Wire.begin();
  Serial.begin(115200);

  WiFi.hostname(HOSTNAME);
  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  mqttClient.setClient(wifiClient);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();

  for (uint8_t i = 0; i < NUM_BMP280; i++) {
    sensors[i] = new PressureSensor(i);
    sensors[i]->connect();
  }

  i2c_select(I2C_DISPLAY);
  lcd.begin();

  lcd.clear();
  lcd.backlight();
  
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connect");

  uint8_t pos = 0;
  while (WiFi.status() != WL_CONNECTED) {
    lcd.setCursor((pos++) % 1, 1);
    lcd.print(" .");

    delay(200);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(10);

  // Setup timers
  timer.setInterval(10 * TIME_SECOND_MS, tryConnectPressureSensors);
  timer.setInterval(300, updateDisplay);

  setSensorUpdate(SENSOR_POLL_INTERVAL_INACTIVE_MS);

  timer.setInterval(TIME_SECOND_MS, []() {
    blink = !blink;
  });

  tryConnectPressureSensors();
  updatePressureSensorValues();
  
  mqttConnect();
}

void setSensorUpdate(long interval) {

  if (updateSensorValuesTimerId != -1) {
    timer.deleteTimer(updateSensorValuesTimerId);
    updateSensorValuesTimerId = -1;
  }

  updateSensorValuesTimerId = timer.setInterval(interval, updatePressureSensorValues);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  const char* charPayload = (char*) payload;

  if (strcmp(topic, MQTT_TOPIC_LASER_OPERATION) == 0) {

    // Delete old timer
    if (laserInactiveTimeoutTimerId != -1) {
      timer.deleteTimer(laserInactiveTimeoutTimerId);
    }
  
    if (strncmp(charPayload, "active", length) == 0) {
      isLaserActive = true;
      setSensorUpdate(SENSOR_POLL_INTERVAL_ACTIVE_MS);
    } else if (strncmp(charPayload, "inactive", length) == 0) {

      // Set back laserActive after LASER_ACTIVE_TIMEOUT_MS seconds, to avoid wrong taras
      laserInactiveTimeoutTimerId = timer.setTimeout(LASER_ACTIVE_TIMEOUT_MS * TIME_SECOND_MS, []() {
        isLaserActive = false;
        laserInactiveTimeoutTimerId = -1;
        setSensorUpdate(SENSOR_POLL_INTERVAL_INACTIVE_MS);
      });
    }
  }
}

void mqttConnect() {
  if (!mqttClient.connected()) {
    if (mqttClient.connect(HOSTNAME, MQTT_TOPIC_STATE, 1, true, "disconnected")) {
      mqttClient.subscribe(MQTT_TOPIC_LASER_OPERATION);
      mqttClient.publish(MQTT_TOPIC_STATE, "connected", true);
    }
  }
}

void publishSensorValue(uint8_t sensor, char* topic, float value, uint8_t precision) {
  sprintf(formatBuffer, "%s/%d/%s", MQTT_TOPIC_SENSOR_BASE, sensor, topic);
  
  dtostrf(value, 4, precision, valueBuffer);
  mqttClient.publish(formatBuffer, valueBuffer);
}

void publishNullMessage(uint8_t sensor, char* topic) {
  sprintf(formatBuffer, "%s/%d/%s", MQTT_TOPIC_SENSOR_BASE, sensor, topic);
  mqttClient.publish(formatBuffer, "", 0);
}

void updateDisplay() {

  i2c_select(I2C_DISPLAY);

  lcd.setCursor(0, 0);
  lcd.print("Sensors: ");

  PressureSensor* selectedSensor = sensors[selectedPressureSensor];

  for (uint8_t i = 0; i < NUM_BMP280; i++) {
    PressureSensor* sensor = sensors[i];

    if (selectedPressureSensor == i && blink) {
      lcd.print(" ");
      continue;
    }

    if (sensor->isConnected()) {
      lcd.print(i + 1);
    } else {
      lcd.print("_");
    }
  }

  lcd.setCursor(0, 1);

  if (selectedSensor->isConnected()) {

    if (showTemperature) {
      lcd.print(selectedSensor->getTemperature());
      lcd.print(" ");
      lcd.print((char) 0xDF);
      lcd.print("C             ");

    } else {
      lcd.print(selectedSensor->getPressure() / 100.0);
      lcd.print(" hPa            ");
    }

  } else {
    lcd.print("Not connected.");
  }
}

void tryConnectPressureSensors() {
  for (uint8_t i = 0; i < NUM_BMP280; i++) {
    sensors[i]->connect();
  }
}

void updatePressureSensorValues() {
  for (uint8_t i = 0; i < NUM_BMP280; i++) {
      PressureSensor* sensor = sensors[i];
      
      bool wasConnectedBefore = sensor->isConnected();
      
      sensor->update(!isLaserActive);

      if (wasConnectedBefore) {

        // Still connected?
        if (sensor->isConnected()) {
          publishSensorValue(i + 1, "pressure", sensor->getPressure() / 100.0, 3);
          publishSensorValue(i + 1, "pressure/relative", sensor->getRelativePressure() / 100.0, 3);
          publishSensorValue(i + 1, "temperature", sensor->getTemperature(), 2);
        } else {
          // Sensor disconnected after update
          publishNullMessage(i + 1, "pressure");
          publishNullMessage(i + 1, "pressure/relative");
          publishNullMessage(i + 1, "temperature");
        }
      }
  }
}


long oldPosition  = -999;

void loop() {
  mqttConnect();

  long newPosition = encoder.read() / 2;
  if (newPosition > oldPosition) {

    if (selectedPressureSensor == 0) {
      selectedPressureSensor = NUM_BMP280 - 1;
    } else {
      selectedPressureSensor--;
    }

    updateDisplay();
  } else if (newPosition < oldPosition) {
    selectedPressureSensor = (selectedPressureSensor + 1) % NUM_BMP280;
    updateDisplay();
  }

  oldPosition = newPosition;;

  if (debouncer.fell()) {
    showTemperature = !showTemperature;
    updateDisplay();
  }

  timer.run();
  debouncer.update();
  mqttClient.loop();
}
