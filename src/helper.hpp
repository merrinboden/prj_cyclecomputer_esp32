#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RtcDS1302.h>
#include <ThreeWire.h>
#include <CoAP-simple.h>
#include <esp_sleep.h>
#include <cstdarg>

// ===== CONFIGURATION CONSTANTS =====
namespace Pins {
  // I2C
  static const uint8_t SDA = 19;
  static const uint8_t SCL = 21;

  // DHT
  static const uint8_t DHT = 32;

  // RGB LED
  static const uint8_t LED_R = 25;
  static const uint8_t LED_G = 26;
  static const uint8_t LED_B = 27;

  // RTC DS1302
  static const uint8_t DS1302_RST = 18;
  static const uint8_t DS1302_DAT = 5; 
  static const uint8_t DS1302_CLK = 4; 

  // Button
  static const uint8_t BTN = 23;
}

namespace I2CAddr {
  static const uint8_t LCD = 0x27;
  static const uint8_t MPU = 0x68; 
}
namespace Config {
  // WiFi configuration
  constexpr char WIFI_SSID[] = "Merrin's Pixel 8";
  constexpr char WIFI_PASSWORD[] = "12345678";
  constexpr uint32_t WIFI_TIMEOUT_MS = 15000;
  
  // CoAP configuration
  constexpr char COAP_SERVER_IP[] = "10.192.116.1";
  constexpr uint16_t COAP_SERVER_PORT = 5683;
  constexpr char COAP_RESOURCE_PATH[] = "/data";
  
  // Timing intervals (ms)
  constexpr uint32_t DHT_READ_MS = 2000;
  // Telemetry timing
  // When connected and moving: send every 2 seconds
  constexpr uint32_t COAP_SEND_MOVING_MS = 2000;  // 2s during movement
  // When idle: send a heartbeat every 10 minutes
  constexpr uint32_t COAP_SEND_IDLE_MS = 600000; // 10 min heartbeat
  constexpr uint32_t ACCEL_SAMPLE_MS = 50;  // 20 Hz accelerometer sampling
  constexpr uint32_t SENSOR_READ_ACTIVE_MS = 500;  // Sensor read interval when active
  constexpr uint32_t SENSOR_READ_IDLE_MS = 2000;   // Sensor read interval when idle
  
  // Power management
  constexpr uint32_t SLEEP_THRESHOLD_MS = 300000; // 10s disconnected = sleep
  constexpr uint32_t MOVEMENT_TIMEOUT_MS = 10000; // 10s no movement = idle
  constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 600000; // 10 min retry when disconnected
  constexpr float MOVEMENT_ACCEL_THRESHOLD = 2.0f; // m/s² for movement detection
  constexpr float MOVEMENT_GYRO_THRESHOLD = 5.0f;  // rad/s for movement detection
  constexpr float MOVEMENT_ACCEL_THEFT_THRESHOLD_X = 12.0f; // m/s² for movement detection
  constexpr float MOVEMENT_ACCEL_THEFT_THRESHOLD_Y = 2.0f; // m/s² for movement detection
  constexpr float MOVEMENT_ACCEL_THEFT_THRESHOLD_Z = 2.0f; // m/s² for movement detection
  constexpr float MOVEMENT_GYRO_THEFT_THRESHOLD = 5.0f;  // rad/s for movement detection

  // Button settings
  constexpr uint8_t TOTAL_PAGES = 5;
  // Button wiring: set to true if the button pulls the pin to GND when pressed
  // (use internal INPUT_PULLUP). Set to false if the button pulls to VCC
  // (use internal INPUT_PULLDOWN).
  constexpr bool BUTTON_PULLUP = true;
  
  // System settings
  constexpr uint32_t BAUD_RATE = 115200;
}

// ===== STATE STRUCTURES =====
struct SensorData {
  float temperature = NAN;
  float humidity = NAN;
  float accel_x = NAN, accel_y = NAN, accel_z = NAN;
  float gyro_x = NAN, gyro_y = NAN, gyro_z = NAN;
  bool dht_ok = false;
  bool mpu_ok = false;
};

struct SystemState {
  bool wifi_connected = false;
  bool locked = false;
  bool theft_detected = false;
  uint32_t coap_transmissions = 0;
  uint32_t last_coap_transmission = 0;
  
  // State Machine state tracking
  uint8_t current_state = 0;  // StateMachine::INIT
  uint8_t previous_state = 0;
  uint32_t state_entry_time = 0;
  uint32_t last_transition = 0;
  uint8_t last_event = 0;
  
  // Power management state
  bool movement_detected = false;
  uint32_t last_movement_time = 0;
  uint32_t wifi_disconnect_time = 0;
  uint32_t last_wifi_retry = 0;
  bool is_moving = false;
  float speed_kmh = 0.0f;
  float elevationChange = 0.0f;
  float movement_magnitude = 0.0f;
  uint8_t ui_page = 0;
  
  // LED state
  uint8_t led_status = 0;  // LEDSystem::INIT
  uint32_t led_change_time = 0;
  uint8_t sos_step = 0;
  bool led_on = false;
  bool button_reset = true;
};

// ===== LED SYSTEM =====
namespace LEDSystem {
  enum Status : uint8_t {
    INIT,            // Blue blink - System initializing
    COAP_ERROR,      // Red - CoAP/WiFi connection issues
    COAP_CONNECTING, // Blue - Connecting to WiFi/CoAP
    SYSTEM_OK,       // Green - Idle/stable/transmitting
    THEFT_ALERT      // Red SOS - Theft detected
  };
  
  constexpr uint16_t INIT_BLINK_INTERVAL_MS = 5000; // 5000ms blink interval

  // SOS pattern timing (3 short, 3 long, 3 short)
  constexpr uint16_t SOS_PATTERN[] = {200, 200, 200, 200, 200, 600, 600, 200, 600, 200, 600, 600, 200, 200, 200, 200, 200, 1400};
  constexpr uint8_t SOS_PATTERN_LENGTH = sizeof(SOS_PATTERN) / sizeof(SOS_PATTERN[0]);
}

// ===== POWER MANAGEMENT =====
namespace PowerMgmt {
  inline void enableWiFiLightSleep() {
    WiFi.setSleep(WIFI_PS_MIN_MODEM); // Light sleep mode
  }
  
  inline void disableWiFiSleep() {
    WiFi.setSleep(WIFI_PS_NONE);
  }

  //set conditions for sleep
  //if state is disconnected, enter deep sleep
}

// ===== BUTTON CONTROL =====
namespace Button {
  inline void init() {
    if (Config::BUTTON_PULLUP) pinMode(Pins::BTN, INPUT_PULLUP);
    else pinMode(Pins::BTN, INPUT_PULLDOWN);
  }
  
  inline bool checkPageChange(SystemState& state) {
    static bool last_button_state = HIGH;
    static uint32_t last_button_change = 0;
    uint32_t now = millis();
    bool current_state = digitalRead(Pins::BTN);
    
    // Check for button press (depends on pull mode)
    // If using pull-up: idle is HIGH, press -> LOW
    // If using pull-down: idle is LOW, press -> HIGH
    bool pressed = false;
    if (Config::BUTTON_PULLUP) {
      pressed = (last_button_state == HIGH && current_state == LOW);
    } else {
      pressed = (last_button_state == LOW && current_state == HIGH);
    }

    if (pressed) {
      state.ui_page = (state.ui_page + 1) % Config::TOTAL_PAGES;
      last_button_change = now;
      last_button_state = current_state;
      Serial.printf("Button pressed - Page changed to: %d (raw=%d)\n", state.ui_page, current_state);
      return true;
    } else if (last_button_state != current_state) {
      // Log raw transitions for debugging (rate-limited by debounce)
      last_button_change = now;
      last_button_state = current_state;
      Serial.printf("Button raw change: %d\n", current_state);
    }
    
    return false;
  }
}

// ===== UTILITY FUNCTIONS =====
namespace Utils {
  // Movement detection based on accelerometer and gyroscope
  inline bool detectMovement(const SensorData& sensors, SystemState& state, uint32_t now) {
    // Calculate movement magnitude
    float accel_mag = sqrtf(sensors.accel_x*sensors.accel_x + sensors.accel_y*sensors.accel_y + sensors.accel_z*sensors.accel_z);
    float gyro_mag = sqrtf(sensors.gyro_x*sensors.gyro_x + sensors.gyro_y*sensors.gyro_y + sensors.gyro_z*sensors.gyro_z);
    
    state.movement_magnitude = accel_mag;
    
    bool movement = (accel_mag > (9.81f + Config::MOVEMENT_ACCEL_THRESHOLD) || 
                    accel_mag < (9.81f - Config::MOVEMENT_ACCEL_THRESHOLD)) ||
                   (gyro_mag > Config::MOVEMENT_GYRO_THRESHOLD);
    
    if (movement) {
      state.last_movement_time = now;
      state.movement_detected = true;
    }
    
    // Check if we're currently in moving state
    state.is_moving = (now - state.last_movement_time) < Config::MOVEMENT_TIMEOUT_MS;
    
    return movement;
  }
  
  // Theft detection based on excessive motion
  inline bool isTheftDetected(const SensorData& sensors) {
    return (fabsf(sensors.accel_x) > Config::MOVEMENT_ACCEL_THEFT_THRESHOLD_X || fabsf(sensors.accel_y) > Config::MOVEMENT_ACCEL_THEFT_THRESHOLD_Y || fabsf(sensors.accel_z) > Config::MOVEMENT_ACCEL_THEFT_THRESHOLD_Z || 
            fabsf(sensors.gyro_x) > Config::MOVEMENT_GYRO_THEFT_THRESHOLD || fabsf(sensors.gyro_y) > Config::MOVEMENT_GYRO_THEFT_THRESHOLD || fabsf(sensors.gyro_z) > Config::MOVEMENT_GYRO_THEFT_THRESHOLD);
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

// ===== WIFI & COAP =====
namespace Network {
  static WiFiUDP udp;
  static Coap coap(udp, 5683);
  static IPAddress server_ip;
  
  inline bool init(SystemState& state) {
    // WiFi init with diagnostics and scan
    Serial.println("Starting WiFi init (diagnostic mode)...");
    WiFi.mode(WIFI_STA);
    // Ensure a clean start
    WiFi.disconnect(true);
    delay(100);

    // Scan for nearby networks to confirm SSID is visible
    Serial.println("Scanning for nearby WiFi networks...");
    int n = WiFi.scanNetworks();
    bool found = false;
    if (n == 0) {
      Serial.println("No networks found (scan returned 0)");
    } else {
      Serial.printf("%d networks found:\n", n);
      for (int i = 0; i < n; ++i) {
        String ss = WiFi.SSID(i);
        int rssi = WiFi.RSSI(i);
        String sec = WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OPEN" : "SEC";
        Serial.printf("  %d: %s (%d dBm) %s\n", i+1, ss.c_str(), rssi, sec.c_str());
        if (ss == String(Config::WIFI_SSID)) found = true;
      }
    }

    if (!found) {
      Serial.println("Warning: target SSID not found in scan results. Check SSID or AP range.");
    }

    Serial.printf("Attempting to connect to SSID '%s'...\n", Config::WIFI_SSID);
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < Config::WIFI_TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
      state.wifi_connected = true;
      state.wifi_disconnect_time = 0;
      server_ip.fromString(Config::COAP_SERVER_IP);
      coap.start();
      PowerMgmt::enableWiFiLightSleep(); // Enable power saving
      return true;
    }

    // Print final status code for diagnostics
    Serial.printf("WiFi failed!");
    switch (WiFi.status()) {
      case WL_NO_SHIELD: Serial.println("  WL_NO_SHIELD"); break;
      case WL_IDLE_STATUS: Serial.println("  WL_IDLE_STATUS"); break;
      case WL_NO_SSID_AVAIL: Serial.println("  WL_NO_SSID_AVAIL (SSID not in range)"); break;
      case WL_SCAN_COMPLETED: Serial.println("  WL_SCAN_COMPLETED"); break;
      case WL_CONNECTED: Serial.println("  WL_CONNECTED"); break;
      case WL_CONNECT_FAILED: Serial.println("  WL_CONNECT_FAILED (auth failure)"); break;
      case WL_CONNECTION_LOST: Serial.println("  WL_CONNECTION_LOST"); break;
      case WL_DISCONNECTED: Serial.println("  WL_DISCONNECTED"); break;
      default: Serial.println("  Unknown status"); break;
    }

    state.wifi_disconnect_time = millis();
    return false;
  }
  
  inline bool sendTelemetry(SystemState& state, const SensorData& sensors, bool is_heartbeat = false) {
    if (!state.wifi_connected || WiFi.status() != WL_CONNECTED) return false;
    
    // Validate server IP
    if (server_ip == INADDR_NONE || server_ip == IPAddress(0,0,0,0)) {
      Serial.println("Invalid server IP, skipping transmission");
      return false;
    }
    
    // Temporarily disable WiFi sleep for transmission
    PowerMgmt::disableWiFiSleep();
    
    // Create compact JSON payload
    char payload[120];
    if (is_heartbeat) {
      snprintf(payload, sizeof(payload),
               "{\"t\":%.1f,\"h\":%.1f,\"hb\":1,\"tx\":%lu}",
               sensors.temperature, sensors.humidity, state.coap_transmissions);
    } else {
      snprintf(payload, sizeof(payload),
               "{\"t\":%.1f,\"h\":%.1f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"mv\":%.2f}",
               sensors.temperature, sensors.humidity,
               state.speed_kmh, state.elevationChange, state.movement_magnitude);
    }
    
    // Use CoAP NON (non-confirmable) for power efficiency
    int msgid = 0;
    try {
      msgid = coap.send(server_ip, Config::COAP_SERVER_PORT, Config::COAP_RESOURCE_PATH, 
                       COAP_NONCON, COAP_POST, NULL, 0, (uint8_t*)payload, strlen(payload));
    } catch (...) {
      Serial.println("CoAP send failed with exception");
      PowerMgmt::enableWiFiLightSleep();
      return false;
    }
    
    if (msgid > 0) {
      state.coap_transmissions++;
      state.last_coap_transmission = millis();
      Serial.printf("CoAP %s: %s\n", is_heartbeat ? "heartbeat" : "data", payload);
      
      // Re-enable WiFi sleep after transmission
      delay(25); // Allow transmission to complete (20ms + margin)
      PowerMgmt::enableWiFiLightSleep();
      return true;
    }
    
    PowerMgmt::enableWiFiLightSleep();
    return false;
  }
  
  inline void maintain(SystemState& state, uint32_t now) {
    bool was_connected = state.wifi_connected;
    state.wifi_connected = (WiFi.status() == WL_CONNECTED);
    
    // Track disconnect time for power management
    if (was_connected && !state.wifi_connected) {
      state.wifi_disconnect_time = now;
      Serial.println("WiFi disconnected - starting disconnect timer");
    } else if (!was_connected && state.wifi_connected) {
      state.wifi_disconnect_time = 0;
      PowerMgmt::enableWiFiLightSleep();
      // On reconnect, ensure CoAP/UDP is (re)started and server IP is set
      server_ip.fromString(Config::COAP_SERVER_IP);
      coap.start();
      Serial.println("WiFi reconnected - CoAP restarted and power saving enabled");
    }
    
    // Attempt reconnection with power-aware timing
    if (!state.wifi_connected && 
        (now - state.last_wifi_retry) > Config::WIFI_RETRY_INTERVAL_MS) {
      Serial.println("Attempting WiFi reconnection...");
      WiFi.reconnect();
      state.last_wifi_retry = now;
    }
    
    // Reduce CoAP loop frequency to prevent UDP errors
    static uint32_t last_coap_loop = 0;
    if ((now - last_coap_loop) > 1000) { // Only call every second
      // Only call coap.loop() when WiFi is connected to avoid repeated
      // UDP parse errors while offline.
      if (state.wifi_connected) {
        coap.loop();
      }
      last_coap_loop = now;
    }
  }
}


// ===== STATE MACHINE =====
namespace StateMachine {
  enum State : uint8_t {
    INIT,           // Initial startup and hardware configuration
    WIFI_CONNECTING, // Connecting to WiFi
    IDLE,           // Connected, no movement, low power mode
    ACTIVE,         // Connected, movement detected, active sensing
    DISCONNECTED,   // WiFi lost, attempting reconnection
    DEEP_SLEEP,     // Deep sleep mode to conserve power
    ERROR           // Error state for recovery
  };
  
  enum Event : uint8_t {
    WIFI_CONNECTED,
    WIFI_LOST,
    MOVEMENT_DETECTED,
    MOVEMENT_STOPPED,
    DEEP_SLEEP_TIMEOUT,
    SENSOR_ERROR,
    BUTTON_PRESSED,
    TIMER_EXPIRED
  };
  
  struct Context {
    State current_state = INIT;
    State previous_state = INIT;
    uint32_t state_entry_time = 0;
    uint32_t last_transition = 0;
    Event last_event = TIMER_EXPIRED;
  };
  
  // Get state name for debugging (must be defined before transition function)
  inline const char* getStateName(State state) {
    switch(state) {
      case INIT: return "INIT";
      case IDLE: return "IDLE";
      case ACTIVE: return "ACTIVE";
      case WIFI_CONNECTING: return "WIFI_CONNECTING";
      case DISCONNECTED: return "DISCONNECTED";
      case DEEP_SLEEP: return "DEEP_SLEEP";
      case ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
  }
  
  // State transition function
  inline void transition(SystemState& state, State new_state, Event event = TIMER_EXPIRED) {
    if (state.current_state != new_state) {
      state.previous_state = state.current_state;
      state.current_state = new_state;
      state.state_entry_time = millis();
      state.last_transition = millis();
      state.last_event = event;
      
      Serial.printf("State: %s -> %s (event: %d)\n", 
                   getStateName((State)state.previous_state),
                   getStateName((State)state.current_state), 
                   event);
    }
  }

  // Main state machine executor
  inline void execute(SystemState& state, SensorData& sensors, uint32_t now) {
    // Update WiFi status
    state.wifi_connected = (WiFi.status() == WL_CONNECTED);
    
    // Check for movement in ACTIVE and IDLE states
    if (state.current_state == ACTIVE || state.current_state == IDLE) {
      if (sensors.mpu_ok) {
        ::Utils::detectMovement(sensors, state, now);
      }
    }
    
    switch(state.current_state) {
      case INIT:
        // Hardware already initialized in setup()
        if (state.wifi_connected) {
          transition(state, IDLE, WIFI_CONNECTED);
        } else if ((now - state.state_entry_time) > Config::WIFI_TIMEOUT_MS) {
          transition(state, DISCONNECTED, TIMER_EXPIRED);
        }
        break;
        
      case IDLE:
        if (!state.wifi_connected) {
          transition(state, DISCONNECTED, WIFI_LOST);
        } else if (state.movement_detected) {
          transition(state, ACTIVE, MOVEMENT_DETECTED);
        }
        break;
        
      case ACTIVE:
        if (!state.wifi_connected) {
          transition(state, DISCONNECTED, WIFI_LOST);
        } else if (!state.movement_detected) {
          // Check if we've been idle long enough
          if ((now - state.last_movement_time) > Config::MOVEMENT_TIMEOUT_MS) {
            transition(state, IDLE, MOVEMENT_STOPPED);
          }
        }
        break;

      case WIFI_CONNECTING:
        if (state.wifi_connected) {
          transition(state, IDLE, WIFI_CONNECTED);
        } else if ((now - state.state_entry_time) > Config::WIFI_TIMEOUT_MS) {
          transition(state, DISCONNECTED, TIMER_EXPIRED);
        }
        break;
        
      case DISCONNECTED:
        if (state.wifi_connected) {
          transition(state, IDLE, WIFI_CONNECTED);
        } else if (!state.locked && (now - state.state_entry_time) > Config::SLEEP_THRESHOLD_MS) {
          transition(state, DEEP_SLEEP, TIMER_EXPIRED);
        }
        break;
        
      case DEEP_SLEEP:
        // Enter deep sleep for power saving
        {
          Serial.println("Entering deep sleep mode...");
          delay(100); // Allow time for message to be sent
          esp_sleep_enable_timer_wakeup(60000000); // Wake up after 60 seconds
          esp_deep_sleep_start();
        }
        break;

      case ERROR:
        // Error recovery - restart after timeout
        Serial.println("Error mode...");
        if ((now - state.state_entry_time) > 30000) { // 30 second error timeout
          transition(state, INIT, TIMER_EXPIRED);
        }
        break;
    }
  }

  inline void updateSensors(SystemState& state, SensorData& sensors, uint32_t now, ::DHT_Unified& dht, ::Adafruit_MPU6050& mpu) {
    // Adaptive sensor reading based on state
    static uint32_t next_sensor_read = 0;
    uint32_t sensor_interval;
    
    switch(state.current_state) {
      case ACTIVE:
        sensor_interval = Config::SENSOR_READ_ACTIVE_MS; // 500ms when active
        break;
      case IDLE:
        sensor_interval = Config::SENSOR_READ_IDLE_MS; // 2s when idle
        break;
      default:
        sensor_interval = Config::SENSOR_READ_IDLE_MS;
        break;
    }
    
    if ((int32_t)(now - next_sensor_read) >= 0) {
      // Read sensors based on availability - sensors passed as parameters
      
      if (sensors.dht_ok) {
        sensors_event_t event;

        dht.temperature().getEvent(&event);
        sensors.temperature = event.temperature;
        
        dht.humidity().getEvent(&event);
        sensors.humidity = event.relative_humidity;
      }
      
      if (sensors.mpu_ok) {
        sensors_event_t accel_event, gyro_event, temp_event;
        if (mpu.getEvent(&accel_event, &gyro_event, &temp_event)) {
          sensors.accel_x = accel_event.acceleration.x;
          sensors.accel_y = accel_event.acceleration.y;
          sensors.accel_z = accel_event.acceleration.z;
          sensors.gyro_x = gyro_event.gyro.x;
          sensors.gyro_y = gyro_event.gyro.y;
          sensors.gyro_z = gyro_event.gyro.z;
        }
      }
      
      next_sensor_read = now + sensor_interval;
    }
  }

  inline void calculateVeloElev(SystemState& state, SensorData& sensors) {
    // Placeholder calculations - replace with actual logic
    // For example, using wheel rotations and time for speed
    state.speed_kmh = 15.0f; // Dummy constant speed // integrate acceleration for real speed
    state.elevationChange = 0.0f;    // Dummy constant elevation // calculate elevation gain from acceleration
  }

  inline void handleTelemetry(SystemState& state, SensorData& sensors, uint32_t now) {
    static uint32_t next_transmission = 0;
    
    if ((int32_t)(now - next_transmission) >= 0) {
      if (state.wifi_connected) {
        bool is_heartbeat = (state.current_state == IDLE);
        ::Network::sendTelemetry(state, sensors, is_heartbeat);

        // Set next transmission based on state
        if (state.current_state == ACTIVE) {
          // When connected and moving: send every Config::COAP_SEND_MOVING_MS (2s)
          next_transmission = now + Config::COAP_SEND_MOVING_MS;
        } else {
          // When idle: send heartbeat every Config::COAP_SEND_IDLE_MS (10min)
          next_transmission = now + Config::COAP_SEND_IDLE_MS;
        }
      }
    }
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
    if (Utils::isTheftDetected(sensors) && state.locked) {
      state.theft_detected = true;
    }
    
    if (state.theft_detected) {
      state.led_status = LEDSystem::THEFT_ALERT;
    } else if (!state.wifi_connected) {
      state.led_status = LEDSystem::INIT;
    } else if (state.wifi_connected && state.coap_transmissions > 0 && (now - state.last_coap_transmission) > 30000) {
      state.led_status = LEDSystem::COAP_ERROR;
    } else {
      state.led_status = LEDSystem::SYSTEM_OK;
    }
    
    // Update LED based on status
    switch (state.led_status) {
      case LEDSystem::INIT:
        // Blue blink
        if ((int32_t)(now - state.led_change_time) >= 0) {
          state.led_on = !state.led_on;
          if (state.led_on) setBlue(); else off();
          state.led_change_time = now + LEDSystem::INIT_BLINK_INTERVAL_MS; // 500ms blink interval
        }
        break;

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

// ===== DISPLAY =====
namespace Display {
  inline void showPage(LiquidCrystal_I2C& lcd, uint8_t page, const SensorData& sensors, const SystemState& state, RtcDS1302<ThreeWire>& rtc) {
    char line1[17], line2[17];
    
    switch (page) {
      case 0: // Time
        {
          RtcDateTime now = rtc.GetDateTime();
          Utils::formatDisplay(line1, "%02u:%02u:%02u", now.Hour(), now.Minute(), now.Second());
          Utils::formatDisplay(line2, "%02u/%02u/%04u", now.Day(), now.Month(), now.Year()); 
        }
        break;
      case 1: // Environment
        Utils::formatDisplay(line1, "T:%.1fC H:%.1f%%", sensors.temperature, sensors.humidity);
        Utils::formatDisplay(line2, "DHT:%s", sensors.dht_ok ? "OK" : "ERROR");
        break;
      case 2: // Motion
        Utils::formatDisplay(line1, "A:%.1f,%.1f,%.1f", sensors.accel_x, sensors.accel_y, sensors.accel_z);
        Utils::formatDisplay(line2, "G:%.1f,%.1f,%.1f", sensors.gyro_x, sensors.gyro_y, sensors.gyro_z);
        break;
      case 3: // Velo
        Utils::formatDisplay(line1, "%.1f km/h", state.speed_kmh);
        Utils::formatDisplay(line2, "%.1f m", state.elevationChange);
        break;
      case 4: // Network & State
        Utils::formatDisplay(line1, "WiFi:%s", state.wifi_connected ? "OK" : "OFF");   
        Utils::formatDisplay(line2, "TX:%lu Mv:%.1f", (unsigned long)state.coap_transmissions, state.movement_magnitude);
        break;
    }
    
    lcd.setCursor(0, 0); lcd.print(line1);
    lcd.setCursor(0, 1); lcd.print(line2);
  }
}