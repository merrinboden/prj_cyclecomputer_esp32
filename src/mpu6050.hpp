#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "pins.hpp"

class Mpu6050 {
public:
  bool begin() {
    // Detect address via WHO_AM_I
    _addr = I2CAddr::MPU;
    uint8_t who = 0;
    // Clock source: PLL with X gyro (0x01)
    write8(0x6B, 0x01);
    delay(10);
    // DLPF to ~42Hz (CONFIG=3), good for accel noise
    write8(0x1A, 0x03);
    // Sample rate divider: base 1kHz -> 100Hz
    write8(0x19, 9);
    // Accel +/-2g
    write8(0x1C, 0x00);
    // INT pin: default active high, push-pull, 50us pulse (0x00)
    write8(0x37, 0x00);
    // Enable Data Ready interrupt
    write8(0x38, 0x01);
    // Clear pending
    uint8_t tmp; read8(0x3A, tmp);
    _ok = true;
    return true;
  }
  bool ok() const { return _ok; }
  uint8_t addr() const { return _addr; }

  bool readAccelG(float &gx, float &gy, float &gz) {
    uint8_t buf[6];
    if (!readBuf(0x3B, buf, 6)) return false;
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    gx = ax / 16384.0f; gy = ay / 16384.0f; gz = az / 16384.0f;
    return true;
  }
private:
  uint8_t _addr = I2CAddr::MPU;
  bool _ok = false;

  void write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
  }
  bool read8(uint8_t reg, uint8_t &val) {
    if (!readBuf(reg, &val, 1)) return false;
    return true;
  }
  bool readBuf(uint8_t start, uint8_t* buf, size_t len) {
    Wire.beginTransmission(_addr);
    Wire.write(start);
    if (Wire.endTransmission(false) != 0) return false;
    size_t n = Wire.requestFrom((int)_addr, (int)len);
    if (n != len) return false;
    for (size_t i=0;i<len;++i) buf[i] = Wire.read();
    return true;
  }
};
