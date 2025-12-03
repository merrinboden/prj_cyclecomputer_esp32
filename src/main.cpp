#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <WiFi.h>
#include <CoAP-simple.h>
#include "helper.hpp"

// === GLOBAL OBJECTS ===
DHT dht(Pins::DHT, DHT22);
Adafruit_MPU6050  mpu;
LiquidCrystal_I2C lcd(Config::LCD_ADDR, 16, 2);
ThreeWire myWire(Pins::DS1302_DAT, Pins::DS1302_CLK, Pins::DS1302_RST);
RtcDS1302<ThreeWire> rtc(myWire);

// === STATE ===
SensorData sensors;
SystemState state;

void setup() {
  Serial.begin(Config::BAUD_RATE);
  Serial.println("ESP32 Cycle Computer v3.0 - State Machine");
  
  // Initialize State Machine
  state.current_state = StateMachine::INIT;
  state.state_entry_time = millis();
  
  // Hardware initialization
  Wire.begin(Pins::SDA, Pins::SCL);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Bike Computer");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  LED::init();
  Button::init();
  
  // Sensor initialization
  dht.begin();
  sensors.mpu_ok = mpu.begin();
  Serial.printf("MPU6050: %s\n", sensors.mpu_ok ? "OK" : "FAILED");
  
  rtc.Begin();
  
  // Network initialization
  Network::init(state);
  
  Serial.printf("Setup complete - State: %s\n", StateMachine::getStateName((StateMachine::State)state.current_state));
  lcd.clear();
}

void loop() {
  uint32_t now = millis();
  
  // === STATE MACHINE EXECUTION ===
  StateMachine::execute(state, sensors, now);
  
  // === STATE-DRIVEN SENSOR UPDATES ===
  StateMachine::updateSensors(state, sensors, now);
  
  // === STATE-DRIVEN TELEMETRY ===
  StateMachine::handleTelemetry(state, sensors, now);
  
  // === UI UPDATES (State-independent) ===
  Button::checkPageChange(state);
  LED::updateStatus(state, sensors, now);
  Display::showPage(lcd, state.ui_page, sensors, state, rtc);
  
  // === NETWORK MAINTENANCE ===
  Network::maintain(state, now);
  
  // === ADAPTIVE DELAY BASED ON STATE ===
  uint32_t delay_ms;
  switch(state.current_state) {
    case StateMachine::ACTIVE:
      delay_ms = 20;   // High responsiveness during movement
      break;
    case StateMachine::IDLE:
      delay_ms = 100;  // Medium responsiveness when idle
      break;
    case StateMachine::DISCONNECTED:
      delay_ms = 1000; // Low power when disconnected
      break;
    default:
      delay_ms = 50;   // Default moderate delay
      break;
  }
  delay(delay_ms);
}