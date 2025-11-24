#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <RtcDS1302.h>
#include <ThreeWire.h>
#include "pins.hpp"

// ===== CONFIGURATION CONSTANTS =====
namespace Config {
  // WiFi / MQTT configuration
  constexpr const char* WIFI_SSID = "Merrin's Pixel 8";
  constexpr const char* WIFI_PASS = "12345678";
  constexpr const char* MQTT_HOST = "10.220.204.1";
  constexpr uint16_t MQTT_PORT = 1883;
  constexpr const char* MQTT_CLIENT_ID = "cyclecomputer";
  constexpr const char* MQTT_TOPIC = "cyclecomputer/telemetry";
  constexpr const char* MQTT_USER = ""; // Optional: set non-empty to use broker auth
  constexpr const char* MQTT_PASS = ""; // Optional: set non-empty to use broker auth
  constexpr uint32_t MQTT_BACKOFF_MS = 30000; // backoff after failed connect
  
  // Timing constants
  constexpr uint32_t DHT_INTERVAL_MS = 2500;
  constexpr uint32_t RTC_INTERVAL_MS = 1000;
  constexpr uint32_t MPU_INTERVAL_MS = 100;
  constexpr uint32_t MQTT_INTERVAL_MS = 5000;
  constexpr uint32_t CONNECTIVITY_CHECK_MS = 5000;
  
  // Motion detection thresholds
  constexpr float ACCEL_THRESHOLD = 0.03f;
  constexpr float GYRO_THRESHOLD = 0.5f;
  
  // UI constants
  constexpr uint8_t UI_PAGES = 4;
  constexpr size_t UI_LINE_LENGTH = 17;
}

// ===== SOS PATTERN =====
namespace SOSPattern {
  constexpr uint16_t pattern[] = {
    // S: dot dot dot
    200, 200, 200, 200, 200, 600, // ... with gaps
    // O: dash dash dash  
    600, 200, 600, 200, 600, 600, // --- with gaps
    // S: dot dot dot
    200, 200, 200, 200, 200, 1400 // ... with long pause before repeat
  };
  constexpr uint8_t PATTERN_LENGTH = 18;
}

// ===== STATE STRUCTURES =====
struct DHTState {
  bool ok = false;
  uint32_t err_count = 0;
  uint32_t err_consec = 0;
  bool have_last = false;
  uint32_t last_ok_ms = 0;
  float last_tC = NAN;
  float last_h = NAN;
  uint32_t recovery_attempts = 0;
};

struct MPUState {
  float last_ax = 0, last_ay = 0, last_az = 0;
  float last_gx = 0, last_gy = 0, last_gz = 0;
};

struct SOSState {
  uint32_t next_change = 0;
  uint8_t state = 0;
  bool led_on = false;
};

struct TimingState {
  uint32_t next_dht = 0;
  uint32_t next_rtc = 0;
  uint32_t next_mpu = 0;
  uint32_t next_mqtt = 0;
  uint32_t next_connect_check = 0;
};

struct UIState {
  char line_dht[Config::UI_LINE_LENGTH] = "DHT Init......";
  char line_dhtErr[Config::UI_LINE_LENGTH] = "DHT Init......";
  char line_time[Config::UI_LINE_LENGTH] = "RTC Init.....";
  char line_date[Config::UI_LINE_LENGTH] = "RTC Init.....";
  char line_aXYZ[Config::UI_LINE_LENGTH] = "MPU Init......";
  char line_gXYZ[Config::UI_LINE_LENGTH] = "MPU Init......";
  char line_ip[Config::UI_LINE_LENGTH] = "WiFi Init.....";
  char line_netw[Config::UI_LINE_LENGTH] = "MQTT Init.....";
  uint8_t page = 0;
};

struct NetworkState {
  bool mqtt_suspended = false;
  uint32_t mqtt_cooldown_until = 0;
};

struct SecurityState {
  bool locked = false;
  bool theft_detected = false;
};

// ===== UTILITY FUNCTIONS =====
namespace Utils {
  // String padding utility
  inline void pad16(char* s) {
    size_t l = strlen(s);
    while (l < 16) {
      s[l++] = ' ';
    }
    s[16] = '\0';
  }

  // I2C scanner utility
  inline void i2c_scan() {
    Serial.println("I2C scan...");
    for (uint8_t a = 1; a < 127; ++a) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        Serial.printf(" - 0x%02X\n", a);
      }
    }
  }

  // DHT read with retry logic
  inline bool dht_read_retry(DHT& dht, float& tC, float& hPct, int attempts = 3, uint32_t gapMs = 60) {
    for (int i = 0; i < attempts; ++i) {
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      if (!isnan(h) && !isnan(t)) {
        tC = t;
        hPct = h;
        return true;
      }
      if (i + 1 < attempts) delay(gapMs);
    }
    return false;
  }

  // Motion detection
  inline bool detect_motion(const MPUState& current, const MPUState& previous) {
    return (fabsf(current.last_ax - previous.last_ax) > Config::ACCEL_THRESHOLD ||
            fabsf(current.last_ay - previous.last_ay) > Config::ACCEL_THRESHOLD ||
            fabsf(current.last_az - previous.last_az) > Config::ACCEL_THRESHOLD ||
            fabsf(current.last_gx - previous.last_gx) > Config::GYRO_THRESHOLD ||
            fabsf(current.last_gy - previous.last_gy) > Config::GYRO_THRESHOLD ||
            fabsf(current.last_gz - previous.last_gz) > Config::GYRO_THRESHOLD);
  }
}

// ===== LED CONTROL =====
namespace LED {
  inline void begin() {
    pinMode(Pins::LED_R, OUTPUT);
    pinMode(Pins::LED_G, OUTPUT);
    pinMode(Pins::LED_B, OUTPUT);
    digitalWrite(Pins::LED_R, LOW);
    digitalWrite(Pins::LED_G, LOW);
    digitalWrite(Pins::LED_B, LOW);
  }

  inline void set(bool r, bool g, bool b) {
    digitalWrite(Pins::LED_R, r ? HIGH : LOW);
    digitalWrite(Pins::LED_G, g ? HIGH : LOW);
    digitalWrite(Pins::LED_B, b ? HIGH : LOW);
  }

  inline void red() { set(true, false, false); }
  inline void blue() { set(false, false, true); }
  inline void green() { set(false, true, false); }
  inline void off() { set(false, false, false); }

  // SOS blinking handler
  inline void handle_sos(SOSState& sos_state, uint32_t now) {
    if ((int32_t)(now - sos_state.next_change) >= 0) {
      if (sos_state.state >= SOSPattern::PATTERN_LENGTH) {
        sos_state.state = 0; // reset pattern
      }
      sos_state.led_on = !sos_state.led_on; // toggle LED state
      sos_state.next_change = now + SOSPattern::pattern[sos_state.state];
      sos_state.state++;
    }
    
    if (sos_state.led_on) {
      red();
    } else {
      off();
    }
  }
}

// ===== WIFI & MQTT HELPERS =====
namespace Network {
  // Non-blocking WiFi connect attempt
  inline void wifi_connect() {
    if (WiFi.status() == WL_CONNECTED) return;
    WiFi.mode(WIFI_STA);
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASS);
  }

  // Non-blocking MQTT connect attempt
  inline void mqtt_connect(PubSubClient& mqtt) {
    if (mqtt.connected()) return;
    if (Config::MQTT_USER && *Config::MQTT_USER) {
      mqtt.connect(Config::MQTT_CLIENT_ID, Config::MQTT_USER, Config::MQTT_PASS);
    } else {
      mqtt.connect(Config::MQTT_CLIENT_ID);
    }
  }
}

// ===== DISPLAY MANAGEMENT =====
namespace Display {
  inline void render(LiquidCrystal_I2C& lcd, const UIState& ui_state, const NetworkState& net_state, PubSubClient& mqtt) {
    char l0[Config::UI_LINE_LENGTH];
    char l1[Config::UI_LINE_LENGTH];
    
    if (ui_state.page == 0) {
      // DHT page
      strncpy(l0, ui_state.line_dht, Config::UI_LINE_LENGTH);
      strncpy(l1, ui_state.line_dhtErr, Config::UI_LINE_LENGTH);
    } else if (ui_state.page == 1) {
      // Time/Date page
      strncpy(l0, ui_state.line_time, Config::UI_LINE_LENGTH);
      strncpy(l1, ui_state.line_date, Config::UI_LINE_LENGTH);
    } else if (ui_state.page == 2) {
      // Motion page
      strncpy(l0, ui_state.line_aXYZ, Config::UI_LINE_LENGTH);
      strncpy(l1, ui_state.line_gXYZ, Config::UI_LINE_LENGTH);
    } else {
      // Network page
      strncpy(l0, ui_state.line_ip, Config::UI_LINE_LENGTH);
      strncpy(l1, ui_state.line_netw, Config::UI_LINE_LENGTH);
    }
    
    lcd.setCursor(0, 0);
    lcd.print(l0);
    lcd.setCursor(0, 1);
    lcd.print(l1);
  }
}