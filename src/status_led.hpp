#pragma once
#include <Arduino.h>
#include "pins.hpp"

class StatusLed {
public:
  static void begin() {
    pinMode(Pins::LED_R, OUTPUT);
    pinMode(Pins::LED_G, OUTPUT);
    pinMode(Pins::LED_B, OUTPUT);
    set(false,false,false);
  }
  static void set(bool r, bool g, bool b) {
    digitalWrite(Pins::LED_R, r ? HIGH : LOW);
    digitalWrite(Pins::LED_G, g ? HIGH : LOW);
    digitalWrite(Pins::LED_B, b ? HIGH : LOW);
  }
  static void setIdle() { set(false,true,false); }
  static void setError() { set(true,false,false); }
  static void setBluePulse(uint32_t ms=60) {
    _blueUntil = millis() + ms;
    _blueActive = true;
  }
  static void update() {
    if (_blueActive) {
      if ((int32_t)(millis() - _blueUntil) < 0) {
        set(false,false,true);
      } else {
        _blueActive = false; setIdle();
      }
    }
  }
  static void forceGreen() { setIdle(); }
  static void forceBlue() { set(false,false,true); }
  static void forceRed() { setError(); }
private:
  static bool _blueActive;
  static uint32_t _blueUntil;
};
