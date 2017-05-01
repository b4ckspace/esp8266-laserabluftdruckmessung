#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SimpleTimer.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Bounce2.h>

#define BUTTON_PIN D4
#define NUM_BMP280 7

#define I2C_DISPLAY 7

#define I2C_ADDR_BMP280 0x76
#define I2C_ADDR_MULTIPLEXER 0x70

#define TIME_SECOND_MS 1000

Encoder encoder(D6, D5);
SimpleTimer timer;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Bounce debouncer = Bounce();

struct {
  Adafruit_BMP280 sensor;
  bool connected;
  uint8_t consecutiveErrors;
  float temperature;
  float pressure;
} pressureSensors[NUM_BMP280];

uint8_t currentSelectedPressureSensor = 6;
bool blink = false;
bool showTemperature = false;

void i2c_select(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(I2C_ADDR_MULTIPLEXER);
  Wire.write(1 << i);
  Wire.endTransmission();
  
  delay(40);
}

void setup() {
  delay(6000);
  Serial.begin(115200);

  noInterrupts();
  {
    i2c_select(I2C_DISPLAY);
    delay(1000);
    
    lcd.begin();  
    delay(1000);
    
    lcd.clear();
    lcd.backlight();
  }
  interrupts();
 
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(10);

  tryConnectPressureSensors();

  // Setup timers
  timer.setInterval(10 * TIME_SECOND_MS, tryConnectPressureSensors);
  timer.setInterval(3 * TIME_SECOND_MS, updatePressureSensorValues);
  timer.setInterval(250, updateDisplay);
  
  timer.setInterval(TIME_SECOND_MS, []() {
     blink = !blink;
  });
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
    }

    if (pressureSensors[i].consecutiveErrors > 3) {
        pressureSensors[i].connected = false;
    }

    pressureSensors[i].temperature = temperature;
    pressureSensors[i].pressure = pressure;
  }
}


long oldPosition  = -999;

void loop() {
  
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
}
