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
RtcDS1302<ThreeWire> Rtc(myWire);

// === STATE ===
SensorData sensors;
SystemState state;

// === TIMING ===
uint32_t next_sensor_read = 0;
uint32_t next_coap_send = 0;
uint32_t next_accel_read = 0;

void setup() {
  Serial.begin(Config::BAUD_RATE);
  Serial.println("ESP32 Cycle Computer v2.0");
  
  // Hardware initialization
  Wire.begin(Pins::SDA, Pins::SCL);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Bike Computer");
  
  LED::init();
  Button::init();
  
  // Sensor initialization
  dht.begin();
  sensors.mpu_ok = mpu.begin();
  Serial.printf("MPU6050: %s\n", sensors.mpu_ok ? "OK" : "FAILED");
  
  Rtc.Begin();
  
  // Network initialization
  Network::init(state);
  
  Serial.println("Setup complete");
  lcd.clear();
}

void loop() {
  uint32_t now = millis();
  
  // === ACCELEROMETER SAMPLING (High frequency for movement detection) ===
  if (state.wifi_connected && (int32_t)(now - next_accel_read) >= 0) {
    if (sensors.mpu_ok) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      sensors.accel_x = a.acceleration.x;
      sensors.accel_y = a.acceleration.y;
      sensors.accel_z = a.acceleration.z;
      sensors.gyro_x = g.gyro.x;
      sensors.gyro_y = g.gyro.y;
      sensors.gyro_z = g.gyro.z;
      
      // Detect movement
      Utils::detectMovement(sensors, state, now);
    }
    next_accel_read = now + Config::ACCEL_SAMPLE_MS;
  }
  
  // === DHT SENSOR READING (Low frequency) ===
  if ((int32_t)(now - next_sensor_read) >= 0) {
    sensors.dht_ok = false;
    for (int attempt = 0; attempt < 2; attempt++) { // Reduced retries for power saving
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      if (!isnan(h) && !isnan(t)) {
        sensors.temperature = t;
        sensors.humidity = h;
        sensors.dht_ok = true;
        break;
      }
      delay(30); // Reduced delay
    }
    next_sensor_read = now + Config::DHT_READ_MS;
  }
  
  // === ADAPTIVE COAP TRANSMISSION ===
  if ((int32_t)(now - next_coap_send) >= 0) {
    if (state.wifi_connected) {
      bool is_heartbeat = !state.is_moving;
      Network::sendTelemetry(state, sensors, is_heartbeat);
      
      // Set next transmission time based on movement state
      if (state.is_moving) {
        next_coap_send = now + Config::COAP_SEND_MOVING_MS; // 2 Hz during movement
        Serial.printf("Movement detected - next send in %dms\n", Config::COAP_SEND_MOVING_MS);
      } else {
        next_coap_send = now + Config::COAP_SEND_IDLE_MS; // 30 min heartbeat
        Serial.printf("Idle state - next heartbeat in %dms\n", Config::COAP_SEND_IDLE_MS);
      }
    }
  }
  
  // === LED STATUS UPDATE ===
  LED::updateStatus(state, sensors, now);
  
  // === BUTTON & DISPLAY UPDATE ===
  Button::checkPageChange(state);
  Display::showPage(lcd, state.ui_page, sensors, state, Rtc);
  
  // === NETWORK MAINTENANCE ===
  Network::maintain(state, now);
  
  // Variable delay based on activity state
  if (state.is_moving) {
    delay(20);  // Shorter delay during movement for responsive detection
  } else {
    delay(200); // Longer delay when idle to save power
  }
}