#pragma once
#include <Arduino.h>
#include "pins.hpp"

struct RtcTime { uint8_t sec, min, hour, date, month, year, wday; bool running; };

class DS1302 {
public:
  static void begin() {
    pinMode(Pins::DS1302_RST, OUTPUT);
    pinMode(Pins::DS1302_CLK, OUTPUT);
    pinMode(Pins::DS1302_DAT, INPUT);
    digitalWrite(Pins::DS1302_RST, LOW);
    digitalWrite(Pins::DS1302_CLK, LOW);
  }

  static RtcTime readTime() {
    RtcTime t{};
    uint8_t raw_sec = readReg(0x81), raw_min = readReg(0x83), raw_hr = readReg(0x85);
    uint8_t raw_date= readReg(0x87), raw_mon = readReg(0x89), raw_wday = readReg(0x8B), raw_yr = readReg(0x8D);
    t.running = (raw_sec & 0x80) == 0;
    t.sec = bcd2dec(raw_sec & 0x7F);
    t.min = bcd2dec(raw_min & 0x7F);
    if (raw_hr & 0x80) {
      uint8_t hr12 = bcd2dec(raw_hr & 0x1F);
      bool pm = raw_hr & 0x20; t.hour = (hr12 % 12) + (pm ? 12 : 0);
    } else t.hour = bcd2dec(raw_hr & 0x3F);
    t.date = bcd2dec(raw_date & 0x3F);
    t.month = bcd2dec(raw_mon & 0x1F);
    t.wday = bcd2dec(raw_wday & 0x07);
    t.year = bcd2dec(raw_yr);
    return t;
  }

  static void setCompileTime() {
    const char* d = __DATE__; const char* t = __TIME__;
    auto mon_to_num = [](const char* m){
      if (strncmp(m, "Jan", 3)==0) return 1; if (strncmp(m, "Feb", 3)==0) return 2;
      if (strncmp(m, "Mar", 3)==0) return 3; if (strncmp(m, "Apr", 3)==0) return 4;
      if (strncmp(m, "May", 3)==0) return 5; if (strncmp(m, "Jun", 3)==0) return 6;
      if (strncmp(m, "Jul", 3)==0) return 7; if (strncmp(m, "Aug", 3)==0) return 8;
      if (strncmp(m, "Sep", 3)==0) return 9; if (strncmp(m, "Oct", 3)==0) return 10;
      if (strncmp(m, "Nov", 3)==0) return 11; return 12; };
    char ms[4]={d[0],d[1],d[2],0};
    RtcTime rt{};
    rt.month = mon_to_num(ms);
    rt.date = (uint8_t)atoi(d+4);
    rt.year = (uint8_t)(atoi(d+7)%100);
    rt.hour = (uint8_t)atoi(t);
    rt.min = (uint8_t)atoi(t+3);
    rt.sec = (uint8_t)atoi(t+6);
    rt.wday = 1; rt.running=true;
    writeTime(rt);
  }

  static void writeTime(const RtcTime &t) {
    writeReg(0x8E, 0x00); // WP off
    writeReg(0x80, dec2bcd(t.sec & 0x7F));
    writeReg(0x82, dec2bcd(t.min & 0x7F));
    writeReg(0x84, dec2bcd(t.hour & 0x3F));
    writeReg(0x86, dec2bcd(t.date & 0x3F));
    writeReg(0x88, dec2bcd(t.month & 0x1F));
    writeReg(0x8A, dec2bcd(t.wday ? (t.wday%8):1));
    writeReg(0x8C, dec2bcd(t.year));
    writeReg(0x8E, 0x80); // WP on
  }

private:
  static inline void delayio(){ delayMicroseconds(1); }
  static inline uint8_t bcd2dec(uint8_t b){ return (b&0x0F) + 10*((b>>4)&0x0F);} 
  static inline uint8_t dec2bcd(uint8_t d){ return (uint8_t)((d%10) | ((d/10)<<4)); }
  static void beginTx(){ digitalWrite(Pins::DS1302_RST,HIGH); delayio(); }
  static void endTx(){ digitalWrite(Pins::DS1302_RST,LOW); delayio(); }
  static void writeByte(uint8_t v){
    pinMode(Pins::DS1302_DAT, OUTPUT);
    for (uint8_t i=0;i<8;++i){
      digitalWrite(Pins::DS1302_DAT, (v&0x01)?HIGH:LOW); // LSB first
      digitalWrite(Pins::DS1302_CLK, HIGH); delayio();
      digitalWrite(Pins::DS1302_CLK, LOW); delayio();
      v >>= 1;
    }
  }
  static uint8_t readByte(){
    uint8_t v=0; pinMode(Pins::DS1302_DAT, INPUT);
    // Ensure CLK starts low
    digitalWrite(Pins::DS1302_CLK, LOW); delayio();
    for (uint8_t i=0;i<8;++i){
      // DS1302 outputs LSB first; data is valid while CLK is low
      int bit = digitalRead(Pins::DS1302_DAT);
      v |= (bit?1:0) << i;
      // Advance to next bit on rising edge
      digitalWrite(Pins::DS1302_CLK, HIGH); delayio();
      digitalWrite(Pins::DS1302_CLK, LOW); delayio();
    }
    return v;
  }
  static uint8_t readReg(uint8_t cmd){
    digitalWrite(Pins::DS1302_CLK, LOW); digitalWrite(Pins::DS1302_RST, LOW);
    beginTx(); writeByte(cmd | 0x01); uint8_t v = readByte(); endTx(); return v;
  }
  static void writeReg(uint8_t cmd, uint8_t val){
    digitalWrite(Pins::DS1302_CLK, LOW); digitalWrite(Pins::DS1302_RST, LOW);
    beginTx(); writeByte(cmd & (uint8_t)~0x01); writeByte(val); endTx();
  }
};
