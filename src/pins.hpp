#pragma once
#include <Arduino.h>

namespace Pins {
  // I2C
  static const uint8_t SDA = 21;
  static const uint8_t SCL = 22;

  // DHT11
  static const uint8_t DHT = 23;

  // MPU6050
  static const uint8_t MPU_INT = 19;

  // RGB LED (assume active HIGH)
  static const uint8_t LED_R = 25;
  static const uint8_t LED_G = 26;
  static const uint8_t LED_B = 27;

  // DS1302 (3-wire)
  static const uint8_t DS1302_RST = 18;
  static const uint8_t DS1302_DAT = 5; 
  static const uint8_t DS1302_CLK = 4; 
}

namespace I2CAddr {
  static const uint8_t LCD = 0x27;     // LCD
  static const uint8_t MPU = 0x68;   // MPU6050
}