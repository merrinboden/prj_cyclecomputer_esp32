#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "pins.hpp"

class Lcd {
public:
  explicit Lcd(uint8_t addr = I2CAddr::LCD) : _addr(addr) {}
  void begin() {
    delay(50);
    // 4-bit init
    write4bits(0x03, false); delay(5);
    write4bits(0x03, false); delayMicroseconds(150);
    write4bits(0x03, false); delay(5);
    write4bits(0x02, false);
    command(0x28); // 2 lines, 5x8
    command(0x0C); // display on, no cursor
    command(0x06); // entry mode
    clear();
  }
  void clear() { command(0x01); delay(2); }
  void setCursor(uint8_t col, uint8_t row) {
    static const uint8_t row_offsets[] = {0x00,0x40,0x14,0x54};
    if (row > 3) row = 3;
    command(0x80 | (col + row_offsets[row]));
  }
  void print(const char* s) { while (*s) write(*s++); }
  void printPadded16(const char* s) {
    char buf[17];
    uint8_t n = 0; while (n < 16 && s[n]) { buf[n] = s[n]; ++n; }
    while (n < 16) buf[n++] = ' ';
    buf[16] = '\0';
    print(buf);
  }
  void setAddress(uint8_t a) { _addr = a; }

private:
  uint8_t _addr;
  static constexpr uint8_t RS = 0x01; // P0
  static constexpr uint8_t RW = 0x02; // P1 (forced low)
  static constexpr uint8_t EN = 0x04; // P2
  static constexpr uint8_t BL = 0x08; // P3

  void iowrite(uint8_t val) {
    val &= (uint8_t)~RW; // force write
    val |= BL;           // backlight on
    Wire.beginTransmission(_addr);
    Wire.write(val);
    Wire.endTransmission();
  }
  void pulse(uint8_t data) {
    iowrite(data | EN);
    delayMicroseconds(1);
    iowrite(data & ~EN);
    delayMicroseconds(50);
  }
  void write4bits(uint8_t nibble, bool rs) {
    uint8_t bits = (nibble & 0x0F);
    uint8_t data = (bits << 4) & 0xF0; // high nibble lines
    if (rs) data |= RS;
    iowrite(data);
    pulse(data);
    delayMicroseconds(40);
  }
  void send(uint8_t v, bool rs) {
    write4bits(v >> 4, rs);
    write4bits(v & 0x0F, rs);
  }
  void command(uint8_t v) { send(v, false); }
  void write(char c) { send(c, true); }
};
