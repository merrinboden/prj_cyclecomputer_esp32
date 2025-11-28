#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <RtcDS1302.h>
#include <ThreeWire.h>
#include <CoAP-simple.h>
#include <cstdarg>
#include "pins.hpp"

// ===== CONFIGURATION CONSTANTS =====
namespace Config {
  // WiFi configuration
  constexpr char WIFI_SSID[] = "Merrin's Pixel 8";
  constexpr char WIFI_PASSWORD[] = "12345678";
  constexpr uint32_t WIFI_TIMEOUT_MS = 15000;
  
  // CoAP configuration
  constexpr char COAP_SERVER_IP[] = "192.168.178.140";
  constexpr uint16_t COAP_SERVER_PORT = 5683;
  constexpr char COAP_RESOURCE_PATH[] = "/data";
  
  // LCD address
  constexpr uint8_t LCD_ADDR = 0x27;
  
  // Timing intervals (ms)
  constexpr uint32_t DHT_READ_MS = 2000;
  constexpr uint32_t COAP_SEND_MS = 5000;
  constexpr uint32_t PAGE_CYCLE_MS = 10000;
  
  // System settings
  constexpr uint32_t BAUD_RATE = 115200;
}

// ===== LED SYSTEM =====
namespace LEDSystem {
  enum Status : uint8_t {
    COAP_ERROR,      // Red - CoAP/WiFi connection issues
    COAP_CONNECTING, // Blue - Connecting to WiFi/CoAP
    SYSTEM_OK,       // Green - Idle/stable/transmitting
    THEFT_ALERT      // Red SOS - Theft detected
  };
  
  // SOS pattern timing (3 short, 3 long, 3 short)
  constexpr uint16_t SOS_PATTERN[] = {150, 150, 150, 150, 150, 400, 450, 400, 450, 400, 450, 150, 150, 150, 150, 150, 1000};
  constexpr uint8_t SOS_PATTERN_LENGTH = sizeof(SOS_PATTERN) / sizeof(SOS_PATTERN[0]);
}

// ===== STATE STRUCTURES =====
struct SensorData {
  float temperature = NAN;
  float humidity = NAN;
  float accel_x = 0, accel_y = 0, accel_z = 0;
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  bool dht_ok = false;
  bool mpu_ok = false;
};

struct SystemState {
  bool wifi_connected = false;
  bool theft_detected = false;
  uint32_t coap_transmissions = 0;
  uint32_t last_coap_transmission = 0;
  uint8_t ui_page = 0;
  LEDSystem::Status led_status = LEDSystem::COAP_CONNECTING;
  uint32_t led_change_time = 0;
  uint8_t sos_step = 0;
  bool led_on = false;
};

// ===== UTILITY FUNCTIONS =====
namespace Utils {
  // Theft detection based on excessive motion
  inline bool isTheftDetected(const SensorData& sensors) {
    return (fabsf(sensors.accel_x) > 2.0f || fabsf(sensors.accel_y) > 2.0f || fabsf(sensors.accel_z) > 2.0f ||
            fabsf(sensors.gyro_x) > 250.0f || fabsf(sensors.gyro_y) > 250.0f || fabsf(sensors.gyro_z) > 250.0f);
  }
  
  // Format display string to 16 characters
  inline void formatDisplay(char* dest, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(dest, 17, format, args);
    va_end(args);
    // Pad to 16 characters
    size_t len = strlen(dest);
    while (len < 16) dest[len++] = ' ';
    dest[16] = '\0';
  }
}

// ===== LED CONTROL =====
namespace LED {
  inline void init() {
    pinMode(Pins::LED_R, OUTPUT);
    pinMode(Pins::LED_G, OUTPUT);
    pinMode(Pins::LED_B, OUTPUT);
    digitalWrite(Pins::LED_R, LOW);
    digitalWrite(Pins::LED_G, LOW);
    digitalWrite(Pins::LED_B, LOW);
  }
  
  inline void setRed() { digitalWrite(Pins::LED_R, HIGH); digitalWrite(Pins::LED_G, LOW); digitalWrite(Pins::LED_B, LOW); }
  inline void setBlue() { digitalWrite(Pins::LED_R, LOW); digitalWrite(Pins::LED_G, LOW); digitalWrite(Pins::LED_B, HIGH); }
  inline void setGreen() { digitalWrite(Pins::LED_R, LOW); digitalWrite(Pins::LED_G, HIGH); digitalWrite(Pins::LED_B, LOW); }
  inline void off() { digitalWrite(Pins::LED_R, LOW); digitalWrite(Pins::LED_G, LOW); digitalWrite(Pins::LED_B, LOW); }
  
  inline void updateStatus(SystemState& state, const SensorData& sensors, uint32_t now) {
    // Determine LED status
    if (Utils::isTheftDetected(sensors)) {
      state.theft_detected = true;
    }
    
    if (state.theft_detected) {
      state.led_status = LEDSystem::THEFT_ALERT;
    } else if (!state.wifi_connected) {
      state.led_status = LEDSystem::COAP_CONNECTING;
    } else if (state.wifi_connected && state.coap_transmissions > 0 && 
               (now - state.last_coap_transmission) > 30000) {
      state.led_status = LEDSystem::COAP_ERROR;
    } else {
      state.led_status = LEDSystem::SYSTEM_OK;
    }
    
    // Update LED based on status
    switch (state.led_status) {
      case LEDSystem::COAP_ERROR:
        setRed();
        break;
      case LEDSystem::COAP_CONNECTING:
        setBlue();
        break;
      case LEDSystem::SYSTEM_OK:
        setGreen();
        break;
      case LEDSystem::THEFT_ALERT:
        // SOS pattern
        if ((int32_t)(now - state.led_change_time) >= 0) {
          if (state.sos_step >= LEDSystem::SOS_PATTERN_LENGTH) state.sos_step = 0;
          state.led_on = !state.led_on;
          if (state.led_on) setRed(); else off();
          state.led_change_time = now + LEDSystem::SOS_PATTERN[state.sos_step++];
        }
        break;
    }
  }
}

// ===== WIFI & COAP =====
namespace Network {
  static Coap coap;
  static IPAddress server_ip;
  
  inline bool init(SystemState& state) {
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
    
    Serial.print("Connecting to WiFi");
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < Config::WIFI_TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
      state.wifi_connected = true;
      server_ip.fromString(Config::COAP_SERVER_IP);
      coap.start();
      return true;
    }
    
    Serial.println("\nWiFi failed!");
    return false;
  }
  
  inline bool sendTelemetry(SystemState& state, const SensorData& sensors) {
    if (!state.wifi_connected || WiFi.status() != WL_CONNECTED) return false;
    
    char payload[150];
    snprintf(payload, sizeof(payload),
             "{\"temp\":%.1f,\"hum\":%.1f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}",
             sensors.temperature, sensors.humidity,
             sensors.accel_x, sensors.accel_y, sensors.accel_z,
             sensors.gyro_x, sensors.gyro_y, sensors.gyro_z);
    
    int msgid = coap.send(server_ip, Config::COAP_SERVER_PORT, Config::COAP_RESOURCE_PATH, 
                         COAP_CON, COAP_POST, NULL, 0, (uint8_t*)payload, strlen(payload));
    
    if (msgid > 0) {
      state.coap_transmissions++;
      state.last_coap_transmission = millis();
      Serial.printf("CoAP sent: %s\n", payload);
      return true;
    }
    return false;
  }
  
  inline void maintain(SystemState& state) {
    state.wifi_connected = (WiFi.status() == WL_CONNECTED);
    coap.loop();
  }
}

// ===== DISPLAY =====
namespace Display {
  inline void showPage(LiquidCrystal_I2C& lcd, uint8_t page, const SensorData& sensors, const SystemState& state, const RtcDS1302<ThreeWire>& rtc) {
    char line1[17], line2[17];
    
    switch (page) {
      case 0: // Environment
        Utils::formatDisplay(line1, "T:%.1fC H:%.1f%%", sensors.temperature, sensors.humidity);
        Utils::formatDisplay(line2, "DHT:%s MPU:%s", sensors.dht_ok ? "OK" : "ERR", sensors.mpu_ok ? "OK" : "ERR");
        break;
      case 1: // Time
        {
          RtcDateTime now = rtc.GetDateTime();
          Utils::formatDisplay(line1, "%02u:%02u:%02u", now.Hour(), now.Minute(), now.Second());
          Utils::formatDisplay(line2, "%02u/%02u/%04u", now.Day(), now.Month(), now.Year());
        }
        break;
      case 2: // Motion
        Utils::formatDisplay(line1, "A:%.1f,%.1f,%.1f", sensors.accel_x, sensors.accel_y, sensors.accel_z);
        Utils::formatDisplay(line2, "G:%.1f,%.1f,%.1f", sensors.gyro_x, sensors.gyro_y, sensors.gyro_z);
        break;
      default: // Network
        Utils::formatDisplay(line1, "WiFi:%s", state.wifi_connected ? "OK" : "OFF");
        Utils::formatDisplay(line2, "CoAP TX:%lu", (unsigned long)state.coap_transmissions);
        break;
    }
    
    lcd.setCursor(0, 0); lcd.print(line1);
    lcd.setCursor(0, 1); lcd.print(line2);
  }
}