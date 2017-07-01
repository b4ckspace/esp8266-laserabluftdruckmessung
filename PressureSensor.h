#ifndef __PRESSURE_SENSOR_H__
#define __PRESSURE_SENSOR_H__

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define I2C_ADDR_BMP280 0x76

void i2c_select(uint8_t i);

class PressureSensor {

    private:
      Adafruit_BMP280 sensor;
      
      uint8_t index;
      uint8_t errorCount;
      
      float temperature;
      float pressureTara;
      float pressure;

      bool connected;

      bool isValidTemperature(float temperature);
      bool isValidPressure(float pressure);
      
      void updateTara();

    public:
      PressureSensor(uint8_t index);
      
      float getTemperature();
      float getPressure();
      float getRelativePressure();

      bool isConnected();
      
      void connect();
      void update(bool laserActive);
    
};

#endif
