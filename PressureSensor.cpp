#include "PressureSensor.h"

PressureSensor::PressureSensor(uint8_t index)
  : index(index), connected(false), pressure(0), temperature(0), pressureTara(0) {

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

void PressureSensor::update(bool updateTara) {

  if (!this->isConnected()) {
    return;
  }

  i2c_select(this->index);

  float temperature = this->sensor.readTemperature();
  float pressure = this->sensor.readPressure();
  
  if (!this->isValidTemperature(temperature) || !this->isValidPressure(pressure)) {
    this->errorCount++;

    if (this->errorCount > 3) {
      this->connected = false;
    }

    return;
  } else {
    this->errorCount = 0;
  }

  this->temperature = temperature;
  this->pressure = pressure;

  if (updateTara) {
    this->updateTara();
  }
}

void PressureSensor::connect() {

  if (this->isConnected()) {
    return;
  }
  
  i2c_select(this->index);
  
  if (this->sensor.begin(I2C_ADDR_BMP280)) {
    this->update(false);
    this->connected = true;
  }
}


void PressureSensor::updateTara() {
  this->pressureTara = this->pressure;
}

bool PressureSensor::isValidTemperature(float temperature) {
   return !(temperature > 60 || temperature < -10);
}

bool PressureSensor::isValidPressure(float pressure) {
   return !(pressure > 180000 || pressure < 40000);
}

