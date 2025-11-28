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
#include "pins.hpp"
#include "helper.hpp"

// === GLOBAL OBJECTS ===
DHT dht(Pins::DHT, DHT11);
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
  
  // === SENSOR READING ===
  if ((int32_t)(now - next_sensor_read) >= 0) {
    // DHT sensor with retry
    sensors.dht_ok = false;
    for (int attempt = 0; attempt < 3; attempt++) {
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      if (!isnan(h) && !isnan(t)) {
        sensors.temperature = t;
        sensors.humidity = h;
        sensors.dht_ok = true;
        break;
      }
      delay(50);
    }
    
    // MPU6050 sensor
    if (sensors.mpu_ok) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      sensors.accel_x = a.acceleration.x;
      sensors.accel_y = a.acceleration.y;
      sensors.accel_z = a.acceleration.z;
      sensors.gyro_x = g.gyro.x;
      sensors.gyro_y = g.gyro.y;
      sensors.gyro_z = g.gyro.z;
    }
    
    next_sensor_read = now + Config::DHT_READ_MS;
  }
  
  // === COAP TRANSMISSION ===
  if ((int32_t)(now - next_coap_send) >= 0) {
    if (state.wifi_connected) {
      Network::sendTelemetry(state, sensors);
    }
    next_coap_send = now + Config::COAP_SEND_MS;
  }
  
  // === LED STATUS UPDATE ===
  LED::updateStatus(state, sensors, now);
  
  // === BUTTON & DISPLAY UPDATE ===
  state.button_reset = Button::checkPageChange(state);
  Display::showPage(lcd, state.ui_page, sensors, state, Rtc);
  
  // === NETWORK MAINTENANCE ===
  Network::maintain(state);
  
  delay(100);
}