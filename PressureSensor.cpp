#include "PressureSensor.h"

PressureSensor::PressureSensor(uint8_t index) : index(index) {
  //
}

float PressureSensor::getTemperature() {
  return this->temperature;
}

float PressureSensor::getPressure() {
  return this->pressure;
}

float PressureSensor::getRelativePressure() {
  return this->pressure - this->pressureTara;
}

bool PressureSensor::isConnected() {
  return this->connected;
}

void PressureSensor::update(bool laserActive) {

  if (!laserActive) {
    this->updateTara();
  }

  float temperature = this->sensor.readTemperature();
  float pressure = this->sensor.readPressure();

  if (temperature <= -42) {
    this->errorCount++;
  } else {
    this->errorCount = 0;
  }

  if (this->errorCount > 3) {
    this->connected = false;
    return;
  }

  this->temperature = temperature;
  this->pressure = pressure;
}

void PressureSensor::connect() {
  if (this->connected) {
    return;
  }

  i2c_select(this->index);
  
  if (this->sensor.begin(I2C_ADDR_BMP280)) {
    this->connected = true;
  }
}


void PressureSensor::updateTara() {
  this->pressureTara = this->pressure;
}

