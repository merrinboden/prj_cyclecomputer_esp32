#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <RtcDS1302.h>
#include <ThreeWire.h>
#include "pins.hpp"

// ===== CoAP PROTOCOL DEFINITIONS =====
namespace CoAP {
  // CoAP Message Types
  enum MessageType : uint8_t {
    CON = 0,  // Confirmable
    NON = 1,  // Non-confirmable  
    ACK = 2,  // Acknowledgment
    RST = 3   // Reset
  };
  
  // CoAP Method Codes
  enum MethodCode : uint8_t {
    GET = 1,
    POST = 2,
    PUT = 3,
    DELETE = 4
  };
  
  // CoAP Response Codes
  enum ResponseCode : uint8_t {
    CREATED = 65,      // 2.01
    DELETED = 66,      // 2.02
    VALID = 67,        // 2.03
    CHANGED = 68,      // 2.04
    CONTENT = 69,      // 2.05
    BAD_REQUEST = 128, // 4.00
    UNAUTHORIZED = 129, // 4.01
    NOT_FOUND = 132,   // 4.04
    METHOD_NOT_ALLOWED = 133, // 4.05
    INTERNAL_ERROR = 160 // 5.00
  };
  
  // CoAP Option Numbers
  enum OptionNumber : uint8_t {
    URI_PATH = 11,
    CONTENT_FORMAT = 12,
    MAX_AGE = 14,
    URI_QUERY = 15,
    OBSERVE = 6
  };
  
  // Content Formats
  enum ContentFormat : uint16_t {
    TEXT_PLAIN = 0,
    APPLICATION_JSON = 50,
    APPLICATION_CBOR = 60
  };
}

// ===== CONFIGURATION CONSTANTS =====
namespace Config {
  // WiFi configuration
  constexpr const char* WIFI_SSID = "YourWiFiSSID";
  constexpr const char* WIFI_PASSWORD = "YourWiFiPassword";
  constexpr uint32_t WIFI_TIMEOUT_MS = 10000;
  
  // CoAP configuration
  constexpr const char* COAP_SERVER_IP = "192.168.1.100"; // CoAP server IP
  constexpr uint16_t COAP_SERVER_PORT = 5683; // Standard CoAP port
  constexpr const char* COAP_RESOURCE_PATH = "telemetry";
  constexpr uint32_t COAP_TRANSMISSION_INTERVAL = 5000; // ms between transmissions
  constexpr uint32_t COAP_BACKOFF_MS = 10000; // backoff after failed transmission
  constexpr uint8_t COAP_VERSION = 1;
  constexpr uint16_t COAP_MAX_MESSAGE_SIZE = 256;
  
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

// ===== LED SYSTEM =====
namespace LEDSystem {
  enum Status : uint8_t {
    OFF = 0,
    STARTUP,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    COAP_TRANSMITTING,
    ERROR_DHT,
    ERROR_WIFI,
    SECURITY_ALERT,
    MOTION_DETECTED,
    SYSTEM_OK
  };
  
  enum Color : uint8_t {
    BLACK = 0,
    RED = 1,
    GREEN = 2,
    BLUE = 4,
    YELLOW = RED | GREEN,
    CYAN = GREEN | BLUE,
    MAGENTA = RED | BLUE,
    WHITE = RED | GREEN | BLUE
  };
  
  enum Pattern : uint8_t {
    SOLID,
    BLINK_SLOW,
    BLINK_FAST,
    PULSE,
    SOS,
    HEARTBEAT
  };
  
  // Pattern timing definitions (in milliseconds)
  constexpr uint16_t BLINK_SLOW_ON = 1000;
  constexpr uint16_t BLINK_SLOW_OFF = 1000;
  constexpr uint16_t BLINK_FAST_ON = 200;
  constexpr uint16_t BLINK_FAST_OFF = 200;
  constexpr uint16_t PULSE_PERIOD = 2000;
  constexpr uint16_t HEARTBEAT_ON = 100;
  constexpr uint16_t HEARTBEAT_OFF = 100;
  constexpr uint16_t HEARTBEAT_PAUSE = 800;
  
  // SOS pattern: S(...)O(---)S(...)
  constexpr uint16_t SOS_PATTERN[] = {
    150, 150, 150, 150, 150, 300,  // S: dot dot dot
    450, 150, 450, 150, 450, 300,  // O: dash dash dash
    150, 150, 150, 150, 150, 1200  // S: dot dot dot + pause
  };
  constexpr uint8_t SOS_PATTERN_LENGTH = 18;
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

struct LEDState {
  LEDSystem::Status current_status = LEDSystem::STARTUP;
  LEDSystem::Color current_color = LEDSystem::BLACK;
  LEDSystem::Pattern current_pattern = LEDSystem::SOLID;
  uint32_t next_change = 0;
  uint8_t pattern_step = 0;
  bool led_on = false;
  uint32_t pattern_start_time = 0;
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

struct CoAPMessage {
  uint8_t version : 2;
  uint8_t type : 2;
  uint8_t token_length : 4;
  uint8_t code;
  uint16_t message_id;
  uint8_t token[8];
  uint8_t* options;
  uint16_t options_length;
  uint8_t* payload;
  uint16_t payload_length;
};

struct WiFiCoAPState {
  bool wifi_connected = false;
  bool coap_server_available = false;
  uint32_t last_transmission = 0;
  uint32_t transmission_count = 0;
  uint32_t connection_attempts = 0;
  uint16_t next_message_id = 1;
  uint32_t next_token = 0x12345678;
  IPAddress server_ip;
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
    setColor(LEDSystem::BLACK);
  }

  inline void setColor(LEDSystem::Color color) {
    digitalWrite(Pins::LED_R, (color & LEDSystem::RED) ? HIGH : LOW);
    digitalWrite(Pins::LED_G, (color & LEDSystem::GREEN) ? HIGH : LOW);
    digitalWrite(Pins::LED_B, (color & LEDSystem::BLUE) ? HIGH : LOW);
  }
  
  inline void off() { setColor(LEDSystem::BLACK); }
  
  // Determine appropriate status based on system state
  inline LEDSystem::Status determineStatus(const DHTState& dht_state, 
                                          const WiFiCoAPState& coap_state,
                                          const SecurityState& security_state,
                                          bool motion_detected = false) {
    // Priority order: Security -> Connectivity -> Sensors -> Normal operation
    if (security_state.theft_detected) {
      return LEDSystem::SECURITY_ALERT;
    }
    
    if (motion_detected) {
      return LEDSystem::MOTION_DETECTED;
    }
    
    if (!coap_state.wifi_connected) {
      return LEDSystem::WIFI_CONNECTING;
    }
    
    if (dht_state.err_consec > 10) {
      return LEDSystem::ERROR_DHT;
    }
    
    if (coap_state.wifi_connected) {
      return LEDSystem::WIFI_CONNECTED;
    }
    
    return LEDSystem::SYSTEM_OK;
  }
  
  // Set LED status with appropriate color and pattern
  inline void setStatus(LEDState& led_state, LEDSystem::Status status, uint32_t now) {
    if (led_state.current_status != status) {
      led_state.current_status = status;
      led_state.pattern_step = 0;
      led_state.next_change = now;
      led_state.pattern_start_time = now;
      led_state.led_on = false;
      
      // Configure color and pattern based on status
      switch (status) {
        case LEDSystem::OFF:
          led_state.current_color = LEDSystem::BLACK;
          led_state.current_pattern = LEDSystem::SOLID;
          break;
        case LEDSystem::STARTUP:
          led_state.current_color = LEDSystem::YELLOW;
          led_state.current_pattern = LEDSystem::BLINK_SLOW;
          break;
        case LEDSystem::WIFI_CONNECTING:
          led_state.current_color = LEDSystem::BLUE;
          led_state.current_pattern = LEDSystem::BLINK_FAST;
          break;
        case LEDSystem::WIFI_CONNECTED:
          led_state.current_color = LEDSystem::GREEN;
          led_state.current_pattern = LEDSystem::SOLID;
          break;
        case LEDSystem::COAP_TRANSMITTING:
          led_state.current_color = LEDSystem::CYAN;
          led_state.current_pattern = LEDSystem::PULSE;
          break;
        case LEDSystem::ERROR_DHT:
          led_state.current_color = LEDSystem::YELLOW;
          led_state.current_pattern = LEDSystem::BLINK_FAST;
          break;
        case LEDSystem::ERROR_WIFI:
          led_state.current_color = LEDSystem::RED;
          led_state.current_pattern = LEDSystem::BLINK_SLOW;
          break;
        case LEDSystem::SECURITY_ALERT:
          led_state.current_color = LEDSystem::RED;
          led_state.current_pattern = LEDSystem::SOS;
          break;
        case LEDSystem::MOTION_DETECTED:
          led_state.current_color = LEDSystem::BLUE;
          led_state.current_pattern = LEDSystem::PULSE;
          break;
        case LEDSystem::SYSTEM_OK:
          led_state.current_color = LEDSystem::GREEN;
          led_state.current_pattern = LEDSystem::HEARTBEAT;
          break;
      }
    }
  }
  
  // Update LED based on current pattern
  inline void update(LEDState& led_state, uint32_t now) {
    if ((int32_t)(now - led_state.next_change) < 0) {
      return; // Not time to change yet
    }
    
    switch (led_state.current_pattern) {
      case LEDSystem::SOLID:
        setColor(led_state.current_color);
        led_state.next_change = now + 1000; // Check again in 1 second
        break;
        
      case LEDSystem::BLINK_SLOW:
        led_state.led_on = !led_state.led_on;
        setColor(led_state.led_on ? led_state.current_color : LEDSystem::BLACK);
        led_state.next_change = now + (led_state.led_on ? LEDSystem::BLINK_SLOW_ON : LEDSystem::BLINK_SLOW_OFF);
        break;
        
      case LEDSystem::BLINK_FAST:
        led_state.led_on = !led_state.led_on;
        setColor(led_state.led_on ? led_state.current_color : LEDSystem::BLACK);
        led_state.next_change = now + (led_state.led_on ? LEDSystem::BLINK_FAST_ON : LEDSystem::BLINK_FAST_OFF);
        break;
        
      case LEDSystem::PULSE:
        // Simple pulse: fade in and out over 2 seconds
        uint32_t pulse_time = (now - led_state.pattern_start_time) % LEDSystem::PULSE_PERIOD;
        if (pulse_time < LEDSystem::PULSE_PERIOD / 2) {
          setColor(led_state.current_color);
        } else {
          setColor(LEDSystem::BLACK);
        }
        led_state.next_change = now + 50; // Update every 50ms for smooth pulse
        break;
        
      case LEDSystem::SOS:
        if (led_state.pattern_step >= LEDSystem::SOS_PATTERN_LENGTH) {
          led_state.pattern_step = 0;
        }
        led_state.led_on = !led_state.led_on;
        setColor(led_state.led_on ? led_state.current_color : LEDSystem::BLACK);
        led_state.next_change = now + LEDSystem::SOS_PATTERN[led_state.pattern_step];
        led_state.pattern_step++;
        break;
        
      case LEDSystem::HEARTBEAT:
        // Double beat pattern: on-off-on-pause
        switch (led_state.pattern_step % 4) {
          case 0: // First beat on
            setColor(led_state.current_color);
            led_state.next_change = now + LEDSystem::HEARTBEAT_ON;
            break;
          case 1: // First beat off
            setColor(LEDSystem::BLACK);
            led_state.next_change = now + LEDSystem::HEARTBEAT_OFF;
            break;
          case 2: // Second beat on
            setColor(led_state.current_color);
            led_state.next_change = now + LEDSystem::HEARTBEAT_ON;
            break;
          case 3: // Long pause
            setColor(LEDSystem::BLACK);
            led_state.next_change = now + LEDSystem::HEARTBEAT_PAUSE;
            break;
        }
        led_state.pattern_step++;
        break;
    }
  }
}

// ===== WIFI & COAP HELPERS =====
namespace WiFiCoAP {
  static WiFiUDP udp;
  static WiFiCoAPState* coap_state_ptr = nullptr;
  
  // CoAP option encoding helper
  inline uint16_t encodeOption(uint8_t* buffer, uint16_t delta, uint16_t length, const uint8_t* value) {
    uint16_t pos = 0;
    
    // Encode option delta and length
    uint8_t header = 0;
    if (delta < 13) {
      header |= (delta << 4);
    } else {
      header |= (13 << 4);
    }
    
    if (length < 13) {
      header |= length;
    } else {
      header |= 13;
    }
    
    buffer[pos++] = header;
    
    // Extended delta
    if (delta >= 13) {
      buffer[pos++] = delta - 13;
    }
    
    // Extended length  
    if (length >= 13) {
      buffer[pos++] = length - 13;
    }
    
    // Option value
    if (length > 0 && value) {
      memcpy(&buffer[pos], value, length);
      pos += length;
    }
    
    return pos;
  }
  
  // Encode CoAP message to binary format
  inline uint16_t encodeMessage(uint8_t* buffer, const CoAPMessage& msg) {
    uint16_t pos = 0;
    
    // Header (4 bytes)
    buffer[pos++] = (msg.version << 6) | (msg.type << 4) | msg.token_length;
    buffer[pos++] = msg.code;
    buffer[pos++] = (msg.message_id >> 8) & 0xFF;
    buffer[pos++] = msg.message_id & 0xFF;
    
    // Token
    if (msg.token_length > 0) {
      memcpy(&buffer[pos], msg.token, msg.token_length);
      pos += msg.token_length;
    }
    
    // Options (simplified - just URI-PATH and Content-Format)
    uint16_t option_delta = 0;
    
    // URI-PATH option
    uint16_t uri_len = strlen(Config::COAP_RESOURCE_PATH);
    uint16_t delta = CoAP::URI_PATH - option_delta;
    pos += encodeOption(&buffer[pos], delta, uri_len, (const uint8_t*)Config::COAP_RESOURCE_PATH);
    option_delta = CoAP::URI_PATH;
    
    // Content-Format option (JSON)
    uint8_t content_format[2] = {0, CoAP::APPLICATION_JSON};
    delta = CoAP::CONTENT_FORMAT - option_delta;
    pos += encodeOption(&buffer[pos], delta, 2, content_format);
    
    // Payload marker
    if (msg.payload_length > 0) {
      buffer[pos++] = 0xFF;
      memcpy(&buffer[pos], msg.payload, msg.payload_length);
      pos += msg.payload_length;
    }
    
    return pos;
  }
  
  // Initialize WiFi connection
  inline bool initWiFi() {
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
    
    Serial.print("Connecting to WiFi");
    uint32_t start_time = millis();
    
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < Config::WIFI_TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    } else {
      Serial.println("\nWiFi connection failed!");
      return false;
    }
  }
  
  // Initialize CoAP over WiFi
  inline bool init(WiFiCoAPState& coap_state) {
    coap_state_ptr = &coap_state;
    
    // Initialize WiFi
    if (!initWiFi()) {
      return false;
    }
    
    coap_state.wifi_connected = true;
    
    // Parse server IP
    coap_state.server_ip.fromString(Config::COAP_SERVER_IP);
    
    // Initialize UDP
    udp.begin(0); // Use random local port
    
    Serial.printf("CoAP client initialized. Server: %s:%d\n", 
                  Config::COAP_SERVER_IP, Config::COAP_SERVER_PORT);
    
    return true;
  }
  
  // Send CoAP message over WiFi UDP
  inline bool sendCoAPMessage(WiFiCoAPState& coap_state, const char* payload) {
    if (!coap_state.wifi_connected || WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected");
      return false;
    }
    
    // Create CoAP message structure
    CoAPMessage msg;
    msg.version = Config::COAP_VERSION;
    msg.type = CoAP::NON;  // Non-confirmable for simplicity
    msg.code = CoAP::POST;
    msg.message_id = coap_state.next_message_id++;
    msg.token_length = 4;
    
    // Generate token
    uint32_t token = coap_state.next_token++;
    msg.token[0] = (token >> 24) & 0xFF;
    msg.token[1] = (token >> 16) & 0xFF;
    msg.token[2] = (token >> 8) & 0xFF;
    msg.token[3] = token & 0xFF;
    
    msg.payload = (uint8_t*)payload;
    msg.payload_length = strlen(payload);
    
    // Encode to binary format
    uint8_t buffer[Config::COAP_MAX_MESSAGE_SIZE];
    uint16_t message_length = encodeMessage(buffer, msg);
    
    // Send over UDP
    udp.beginPacket(coap_state.server_ip, Config::COAP_SERVER_PORT);
    udp.write(buffer, message_length);
    bool success = udp.endPacket();
    
    if (success) {
      Serial.printf("CoAP message sent: ID=%u, Token=0x%08X, Size=%u bytes\n", 
                    msg.message_id, token, message_length);
      Serial.printf("Payload: %s\n", payload);
    } else {
      Serial.println("Failed to send CoAP message");
    }
    
    return success;
  }
  
  // Check WiFi connection status
  inline bool isConnected() {
    return WiFi.status() == WL_CONNECTED;
  }
  
  // Reconnect WiFi if needed
  inline void maintain() {
    if (WiFi.status() != WL_CONNECTED && coap_state_ptr) {
      Serial.println("WiFi connection lost. Attempting reconnection...");
      coap_state_ptr->wifi_connected = false;
      coap_state_ptr->connection_attempts++;
      
      if (initWiFi()) {
        coap_state_ptr->wifi_connected = true;
        Serial.println("WiFi reconnected successfully");
      }
    }
  }
}

// ===== DISPLAY MANAGEMENT =====
namespace Display {
  inline void render(LiquidCrystal_I2C& lcd, const UIState& ui_state, const WiFiCoAPState& coap_state) {
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