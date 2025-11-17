#pragma once
#include <Arduino.h>
#include "pins.hpp"

struct Dht11Data { int humidity = -1; int temperature = -1; };

class Dht11 {
public:
  static bool read(Dht11Data &out) {
    uint8_t data[5] = {0,0,0,0,0};

    // Start signal per spec: pull low for at least 18 ms
    pinMode(Pins::DHT, OUTPUT);
    digitalWrite(Pins::DHT, LOW);
    delay(20);
    // Pull high for 20-40 us
    digitalWrite(Pins::DHT, HIGH);
    delayMicroseconds(30);
    pinMode(Pins::DHT, INPUT_PULLUP);

    // Wait for DHT response: low 80us, high 80us
    if (!waitLevel(LOW, 100)) return false;   // sensor pulls LOW
    if (!waitLevel(HIGH, 100)) return false;  // then releases to HIGH
    if (!waitLevel(LOW, 100)) return false;   // start of first bit low

    // Read 40 bits
    for (int i = 0; i < 40; ++i) {
      // Each bit: LOW 50us
      if (!waitLevel(HIGH, 80)) return false; // wait until LOW ends -> HIGH starts
      uint32_t tStart = micros();
      if (!waitLevel(LOW, 100)) return false; // measure length of HIGH pulse
      uint32_t highUs = micros() - tStart;
      data[i/8] <<= 1;
      // Threshold per spec: ~26-28us for 0, ~70us for 1. Use 50us threshold
      if (highUs > 50) data[i/8] |= 1;
    }

    uint8_t sum = (uint8_t)(data[0]+data[1]+data[2]+data[3]);
    if (sum != data[4]) return false;
    out.humidity = data[0];
    out.temperature = data[2];
    return true;
  }
private:
  static bool waitLevel(int targetLevel, uint32_t timeoutUs) {
    uint32_t t = micros();
    while (digitalRead(Pins::DHT) != targetLevel) {
      if ((micros() - t) > timeoutUs) return false;
    }
    return true;
  }
};
