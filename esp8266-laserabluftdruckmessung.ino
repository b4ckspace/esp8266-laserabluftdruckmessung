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


SimpleTimer timer;
PubSubClient mqttClient;
WiFiClient wifiClient;

Encoder encoder(D6, D5);
LiquidCrystal_I2C lcd(0x27, 16, 2); 

PressureSensor pressureSensors[NUM_BMP280];

Bounce debouncer = Bounce();

uint8_t currentSelectedPressureSensor = 6;
bool blink = false;
bool showTemperature = false;
int updateSensorValuesTimerId = -1;

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
  
  Serial.begin(115200);
  Wire.begin();

  WiFi.hostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
  mqttClient.setClient(wifiClient);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  i2c_select(I2C_DISPLAY);  
  lcd.begin();
  
  lcd.clear();
  lcd.backlight();
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(10);

  tryConnectPressureSensors();

  // Setup timers
  timer.setInterval(10 * TIME_SECOND_MS, tryConnectPressureSensors);
  timer.setInterval(100, updateDisplay);

  setSensorUpdate(SENSOR_POLL_INTERVAL_INACTIVE_MS);
  
  timer.setInterval(TIME_SECOND_MS, []() {
     blink = !blink;
  });

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
    if (strncmp(charPayload, "active", length) == 0) {
      setSensorUpdate(SENSOR_POLL_INTERVAL_ACTIVE_MS);
    } else if (strncmp(charPayload, "inactive", length) == 0) {
      setSensorUpdate(SENSOR_POLL_INTERVAL_INACTIVE_MS);
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

void publishSensorValue(char* topic, float value, uint8_t precision) {
   dtostrf(value, 4, precision, valueBuffer);
   mqttClient.publish(topic, valueBuffer);
}

void publishNullMessage(char* topic) {
  mqttClient.publish(topic, "", 0);
}

void updateDisplay() {
  
  i2c_select(I2C_DISPLAY);
  
  lcd.setCursor(0, 0);
  lcd.print("Sensors: ");

  for (uint8_t i = 0; i < NUM_BMP280; i++) {

    if (currentSelectedPressureSensor == i && blink) {
      lcd.print(" ");
      continue;
    }
    
    if (pressureSensors[i].connected) {
      lcd.print(i + 1);
    } else {
      lcd.print("_");
    }
  }

  lcd.setCursor(0, 1);
  
  if (pressureSensors[currentSelectedPressureSensor].connected) {

    if (showTemperature) {
      lcd.print(pressureSensors[currentSelectedPressureSensor].temperature);
      lcd.print(" ");
      lcd.print((char) 0xDF);
      lcd.print("C             ");
      
    } else {
      lcd.print(pressureSensors[currentSelectedPressureSensor].pressure / 100.0);
      lcd.print(" hPa            ");
    }
    
  } else {
    lcd.print("Not connected.");
  }
}

void tryConnectPressureSensors() {
  for (uint8_t i = 0; i < NUM_BMP280; i++) {

    if (pressureSensors[i].connected) {
      continue;
    }

    i2c_select(i);
    delay(5);
    
    if (pressureSensors[i].sensor.begin(I2C_ADDR_BMP280)) {
      Serial.print("Sensor #");
      Serial.print(i);
      Serial.println(" reconnected");
      
      pressureSensors[i].connected = true;
    }
    
    delay(5);
  }
}

void updatePressureSensorValues() {
  for (uint8_t i = 0; i < NUM_BMP280; i++) {
    i2c_select(i);

    if (!pressureSensors[i].connected) {
      continue;
    }

    float temperature = pressureSensors[i].sensor.readTemperature();
    float pressure = pressureSensors[i].sensor.readPressure();

    if (temperature <= -42) {
      pressureSensors[i].consecutiveErrors++;
    } else {
      pressureSensors[i].consecutiveErrors = 0;
      pressureSensors[i].temperature = temperature;
      pressureSensors[i].pressure = pressure;
  
      sprintf(formatBuffer, "%s/%d", MQTT_TOPIC_SENSOR_PRESSURE_BASE, i + 1);
      publishSensorValue(formatBuffer, pressure / 100.0, 3);
      
      sprintf(formatBuffer, "%s/%d", MQTT_TOPIC_SENSOR_TEMPERATURE_BASE, i + 1);
      publishSensorValue(formatBuffer, temperature, 2);
    }

    if (pressureSensors[i].consecutiveErrors > 3) {
        pressureSensors[i].connected = false;

        sprintf(formatBuffer, "%s/%d", MQTT_TOPIC_SENSOR_PRESSURE_BASE, i + 1);
        publishNullMessage(formatBuffer);

        sprintf(formatBuffer, "%s/%d", MQTT_TOPIC_SENSOR_TEMPERATURE_BASE, i + 1);
        publishNullMessage(formatBuffer);
    }
  }
}


long oldPosition  = -999;

void loop() {
  mqttConnect();
  
  long newPosition = encoder.read() / 2;
  if (newPosition > oldPosition) {
    
    if (currentSelectedPressureSensor == 0) {
      currentSelectedPressureSensor = NUM_BMP280 - 1;
    } else {
      currentSelectedPressureSensor--;
    }
    
  } else if (newPosition < oldPosition) {
    currentSelectedPressureSensor = (currentSelectedPressureSensor + 1) % NUM_BMP280;
  }

  oldPosition = newPosition;;

  if (debouncer.fell()) {
    showTemperature = !showTemperature;
  }
  
  timer.run();
  debouncer.update();
  mqttClient.loop();
}
