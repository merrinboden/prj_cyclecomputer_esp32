#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <RtcDS1302.h>
#include <ThreeWire.h>
#include "pins.hpp"

// ===== CONFIGURATION CONSTANTS =====
namespace Config {
  // BLE / CoAP configuration
  constexpr const char* BLE_DEVICE_NAME = "CycleComputer";
  constexpr const char* BLE_SERVICE_UUID = "12345678-1234-5678-9abc-def012345678";
  constexpr const char* BLE_CHARACTERISTIC_UUID = "87654321-4321-8765-cba9-fedcba987654";
  constexpr const char* COAP_RESOURCE_PATH = "/telemetry";
  constexpr uint32_t BLE_ADVERTISING_INTERVAL = 1000; // ms between advertisements
  constexpr uint32_t COAP_BACKOFF_MS = 10000; // backoff after failed transmission
  
  // Timing constants
  constexpr uint32_t DHT_INTERVAL_MS = 2500;
  constexpr uint32_t RTC_INTERVAL_MS = 1000;
  constexpr uint32_t MPU_INTERVAL_MS = 100;
  constexpr uint32_t COAP_INTERVAL_MS = 5000;
  constexpr uint32_t BLE_CHECK_MS = 5000;
  
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
  uint32_t next_coap = 0;
  uint32_t next_ble_check = 0;
};

struct UIState {
  char line_dht[Config::UI_LINE_LENGTH] = "DHT Init......";
  char line_dhtErr[Config::UI_LINE_LENGTH] = "DHT Init......";
  char line_time[Config::UI_LINE_LENGTH] = "RTC Init.....";
  char line_date[Config::UI_LINE_LENGTH] = "RTC Init.....";
  char line_aXYZ[Config::UI_LINE_LENGTH] = "MPU Init......";
  char line_gXYZ[Config::UI_LINE_LENGTH] = "MPU Init......";
  char line_ble[Config::UI_LINE_LENGTH] = "BLE Init......";
  char line_coap[Config::UI_LINE_LENGTH] = "CoAP Init.....";
  uint8_t page = 0;
};

struct BLEState {
  bool connected = false;
  bool advertising = false;
  uint32_t last_transmission = 0;
  uint32_t transmission_count = 0;
  uint32_t connection_count = 0;
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
    Serial.printf("DHT retry: pin=%d, attempts=%d\n", Pins::DHT, attempts);
    for (int i = 0; i < attempts; ++i) {
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      Serial.printf("DHT attempt %d: h=%.1f, t=%.1f, h_nan=%d, t_nan=%d\n", 
                    i+1, h, t, isnan(h), isnan(t));
      if (!isnan(h) && !isnan(t)) {
        tC = t;
        hPct = h;
        Serial.printf("DHT SUCCESS: T=%.1fC, H=%.1f%%\n", t, h);
        return true;
      }
      if (i + 1 < attempts) {
        Serial.printf("DHT retry %d failed, waiting %lums\n", i+1, (unsigned long)gapMs);
        delay(gapMs);
      }
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

// ===== BLE & COAP HELPERS =====
namespace BLECoAP {
  static BLEServer* pServer = nullptr;
  static BLECharacteristic* pCharacteristic = nullptr;
  static bool deviceConnected = false;
  
  class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE client connected");
    };
    
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE client disconnected");
      pServer->startAdvertising(); // restart advertising
    }
  };
  
  // Initialize BLE server
  inline void init() {
    BLEDevice::init(Config::BLE_DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    BLEService *pService = pServer->createService(Config::BLE_SERVICE_UUID);
    
    pCharacteristic = pService->createCharacteristic(
                        Config::BLE_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(Config::BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE server initialized and advertising started");
  }
  
  // Send CoAP-like data over BLE
  inline bool sendData(const char* payload) {
    if (!deviceConnected || !pCharacteristic) {
      return false;
    }
    
    // Simple CoAP-like message format: "POST /telemetry {data}"
    char coapMsg[256];
    snprintf(coapMsg, sizeof(coapMsg), "POST %s %s", Config::COAP_RESOURCE_PATH, payload);
    
    pCharacteristic->setValue(coapMsg);
    pCharacteristic->notify();
    
    Serial.printf("CoAP over BLE sent: %s\n", coapMsg);
    return true;
  }
  
  // Check connection status
  inline bool isConnected() {
    return deviceConnected;
  }
}

// ===== DISPLAY MANAGEMENT =====
namespace Display {
  inline void render(LiquidCrystal_I2C& lcd, const UIState& ui_state, const BLEState& ble_state) {
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
      // BLE/CoAP page
      strncpy(l0, ui_state.line_ble, Config::UI_LINE_LENGTH);
      strncpy(l1, ui_state.line_coap, Config::UI_LINE_LENGTH);
    }
    
    lcd.setCursor(0, 0);
    lcd.print(l0);
    lcd.setCursor(0, 1);
    lcd.print(l1);
  }
}