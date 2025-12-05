#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <WiFi.h>
#include <CoAP-simple.h>
#include "helper.hpp"

// === GLOBAL OBJECTS ===
DHT_Unified dht(Pins::DHT, DHT22);
Adafruit_MPU6050  mpu;
LiquidCrystal_I2C lcd(I2CAddr::LCD, 16, 2);
ThreeWire myWire(Pins::DS1302_DAT, Pins::DS1302_CLK, Pins::DS1302_RST);
RtcDS1302<ThreeWire> rtc(myWire);

// === STATE ===
SensorData sensors;
SystemState state;

// Parse build date/time macros and return an RtcDateTime
static RtcDateTime buildDateTimeFromMacros() {
  char monthStr[4] = {0};
  int day = 0, year = 0;
  sscanf(__DATE__, "%3s %d %d", monthStr, &day, &year);

  int month = 1;
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  for (int i = 0; i < 12; ++i) {
    if (strncmp(monthStr, months + i*3, 3) == 0) { month = i+1; break; }
  }

  int hh = 0, mm = 0, ss = 0;
  sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);

  return RtcDateTime(year, month, day, hh, mm, ss);
}

void setup() {
  Serial.begin(Config::BAUD_RATE);
  Serial.println("ESP32 Cycle Computer Startup...");
  
  // Initialize State Machine
  state.current_state = StateMachine::INIT;
  state.state_entry_time = millis();
  
  // Hardware initialization
  Wire.begin(Pins::SDA, Pins::SCL);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Cycle Computer");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  LED::init();
  Button::init();
  
  // Sensor initialization
  dht.begin();
  sensor_t sensor;

  dht.temperature().getSensor(&sensor);
  // Give DHT time to stabilize and do a first read to mark availability
  delay(200);
  sensors_event_t dht_event;
  dht.temperature().getEvent(&dht_event);
  sensors.dht_ok = !isnan(dht_event.temperature);
  Serial.printf("DHT22: %s\n", sensors.dht_ok ? "OK" : "FAILED");

  sensors.mpu_ok = mpu.begin();
  Serial.printf("MPU6050: %s\n", sensors.mpu_ok ? "OK" : "FAILED");
  
  rtc.Begin();

  // Reset RTC to build/upload time so firmware upload synchronizes device time
  RtcDateTime buildDt = buildDateTimeFromMacros();
  rtc.SetDateTime(buildDt);
  Serial.printf("RTC set to build time: %02u:%02u:%02u %02u/%02u/%04u\n",
                buildDt.Hour(), buildDt.Minute(), buildDt.Second(), buildDt.Day(), buildDt.Month(), buildDt.Year());
  
  // Network initialization
  Network::init(state);
  
  Serial.printf("Setup complete - State: %s\n", StateMachine::getStateName((StateMachine::State)state.current_state));
  lcd.clear();
}

void loop() {
  // === ADAPTIVE DELAY BASED ON STATE ===
  uint32_t now = millis();
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
  
  vTaskDelay(delay_ms / portTICK_PERIOD_MS);
  
  // === STATE MACHINE EXECUTION ===
  StateMachine::execute(state, sensors, now);
  
  // === STATE-DRIVEN SENSOR UPDATES ===
  StateMachine::updateSensors(state, sensors, now, dht, mpu);
  
  // === STATE-DRIVEN TELEMETRY ===
  StateMachine::handleTelemetry(state, sensors, now);
  
  // === UI UPDATES (State-independent) ===
  if (Button::checkPageChange(state)) {
    // Page changed, can add additional handling if needed
  };
  LED::updateStatus(state, sensors, now);
  Display::showPage(lcd, state.ui_page, sensors, state, rtc);
  
  // === NETWORK MAINTENANCE ===
  Network::maintain(state, now);
  
  // Debug timing
  //Serial.println(millis()- now);
}