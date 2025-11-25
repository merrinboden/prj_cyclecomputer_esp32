#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include "pins.hpp"
#include "helper.hpp"
#include <stdint.h>

static LiquidCrystal_I2C lcd(I2CAddr::LCD, 16, 2);
static DHT dht(Pins::DHT, DHT11);
static Adafruit_MPU6050 mpu;
static bool mpu_ok = false;
static ThreeWire ds1302Bus(Pins::DS1302_DAT, Pins::DS1302_CLK, Pins::DS1302_RST);
static RtcDS1302<ThreeWire> Rtc(ds1302Bus);

// Hardware instances
static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);

// Application state
static DHTState dht_state;
static MPUState mpu_state;
static SOSState sos_state;
static TimingState timing;
static UIState ui_state;
static NetworkState net_state;
static SecurityState security_state;

// Helper function for initial RTC setup
static void setup_rtc() {
  Preferences prefs;
  prefs.begin("app", false);
  String buildSig = String(__DATE__) + " " + String(__TIME__);
  String prevSig = prefs.getString("buildSig", "");
  if (prevSig != buildSig) {
    RtcDateTime dt(__DATE__, __TIME__);
    Rtc.SetDateTime(dt);
    prefs.putString("buildSig", buildSig);
  }
  prefs.end();
}

void setup() {
  Serial.begin(115200); delay(300);
  Wire.begin(Pins::SDA, Pins::SCL); Wire.setClock(100000);
  LED::begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print("CycleComputer  ");
  lcd.setCursor(0,1); lcd.print("System Init... ");

  // DHT sensor initialization
  Serial.printf("Initializing DHT11 on pin %d\n", Pins::DHT);
  
  // Ensure pin is properly configured
  pinMode(Pins::DHT, INPUT_PULLUP);
  delay(100);
  
  dht.begin();
  delay(2000); // DHT11 needs time to stabilize after power-on
  Serial.println("DHT11 initialization complete");
  
  // Test immediate reading
  float test_h = dht.readHumidity();
  float test_t = dht.readTemperature();
  Serial.printf("Initial DHT test: t=%.1f, h=%.1f (nan check: t=%d, h=%d)\n", 
                test_t, test_h, isnan(test_t), isnan(test_h));
  
  Rtc.Begin();

  // Set RTC to compile time once per firmware upload using NVS build signature
  setup_rtc();

  // MPU init (polling)
  mpu_ok = mpu.begin(I2CAddr::MPU);
  if (mpu_ok) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }
  Serial.printf("MPU6050 %s at 0x%02X\n", mpu_ok?"OK":"N/A", I2CAddr::MPU);
  
  // I2C device scan for debugging
  Utils::i2c_scan();

  // WiFi + MQTT init
  // Start asynchronous WiFi attempt; skip MQTT connect here to prevent blocking before sensors start.
  Network::wifi_connect();
  mqtt.setServer(Config::MQTT_HOST, Config::MQTT_PORT);
  mqtt.setSocketTimeout(1);   // seconds; keep connect() from blocking long
  mqtt.setKeepAlive(30);

  // Button
  pinMode(Pins::BTN, INPUT_PULLUP);
}

void loop() {
  uint32_t now = millis();

  // DHT11 every 2500 ms (aligned, drift-free)
  if ((int32_t)(now - timing.next_dht) >= 0) {
    if (timing.next_dht == 0) timing.next_dht = now; // initial immediate run
    // maintain alignment even if we overrun: add interval until next_dht in future
    do { timing.next_dht += Config::DHT_INTERVAL_MS; } while ((int32_t)(now - timing.next_dht) >= 0);
    
    bool ok = false;
    float h = NAN, t = NAN;
    
    ok = Utils::dht_read_retry(dht, t, h, 2, 40);
    
    // Debug DHT readings
    Serial.printf("DHT read attempt: ok=%d, t=%.1f, h=%.1f\n", ok, t, h);
    
    dht_state.ok = ok;
    if (!ok) {
      dht_state.err_count++;
      dht_state.err_consec++;
      Serial.printf("DHT failed: consecutive=%lu, total=%lu\n", 
                    (unsigned long)dht_state.err_consec, (unsigned long)dht_state.err_count);
    } else {
      dht_state.err_consec = 0;
      dht_state.last_tC = t; dht_state.last_h = h; dht_state.have_last = true;
    }
    // Prepare DHT UI lines
    if (ok) {
      snprintf(ui_state.line_dht, sizeof(ui_state.line_dht), "T:%dC H:%d%%", (int)roundf(t), (int)roundf(h));
      snprintf(ui_state.line_dhtErr, sizeof(ui_state.line_dhtErr), "DHT OK");
    } else if (dht_state.have_last) {
      snprintf(ui_state.line_dht, sizeof(ui_state.line_dht), "T:%dC H:%d%% (old)", (int)roundf(dht_state.last_tC), (int)roundf(dht_state.last_h));
      snprintf(ui_state.line_dhtErr, sizeof(ui_state.line_dhtErr), "DHT ERR %lu", (unsigned long)dht_state.err_count);
    } else {
      snprintf(ui_state.line_dht, sizeof(ui_state.line_dht), "DHT No Data");
      snprintf(ui_state.line_dhtErr, sizeof(ui_state.line_dhtErr), "DHT ERR %lu", (unsigned long)dht_state.err_count);
    }
    Utils::pad16(ui_state.line_dht);
    Utils::pad16(ui_state.line_dhtErr);
    if (ui_state.page == 0) Display::render(lcd, ui_state, net_state, mqtt);
  }

  // MPU read every 100ms aligned
  bool mpu_event=false;
  if (mpu_ok && (int32_t)(now - timing.next_mpu) >= 0) {
    if (timing.next_mpu == 0) timing.next_mpu = now; do { timing.next_mpu += Config::MPU_INTERVAL_MS; } while ((int32_t)(now - timing.next_mpu) >= 0);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    MPUState current_mpu;
    current_mpu.last_ax = a.acceleration.x / 9.80665f;
    current_mpu.last_ay = a.acceleration.y / 9.80665f;
    current_mpu.last_az = a.acceleration.z / 9.80665f;
    current_mpu.last_gx = g.gyro.x;
    current_mpu.last_gy = g.gyro.y;
    current_mpu.last_gz = g.gyro.z;

    if (Utils::detect_motion(current_mpu, mpu_state)) {
      mpu_event = true;
      mpu_state = current_mpu;
      Serial.printf("MPU motion: a=(%.1f,%.1f,%.1f)g g=(%.1f,%.1f,%.1f)deg/s\n", 
                    current_mpu.last_ax, current_mpu.last_ay, current_mpu.last_az, 
                    current_mpu.last_gx, current_mpu.last_gy, current_mpu.last_gz);
    }
    
    // Update MPU display lines (always update to show current values)
    snprintf(ui_state.line_aXYZ, sizeof(ui_state.line_aXYZ), "A:%.1f,%.1f,%.1f", current_mpu.last_ax, current_mpu.last_ay, current_mpu.last_az);
    Utils::pad16(ui_state.line_aXYZ);
    snprintf(ui_state.line_gXYZ, sizeof(ui_state.line_gXYZ), "G:%.1f,%.1f,%.1f", current_mpu.last_gx, current_mpu.last_gy, current_mpu.last_gz);
    Utils::pad16(ui_state.line_gXYZ);
    
    // Refresh motion page either on detected motion change or every 500ms while displayed
    if (ui_state.page == 2 && (mpu_event)) {
      Display::render(lcd, ui_state, net_state, mqtt);
    }
  }

  // Connectivity check
  if ((int32_t)(now - timing.next_connect_check) >= 0) {
    if (timing.next_connect_check == 0) timing.next_connect_check = now; do { timing.next_connect_check += Config::CONNECTIVITY_CHECK_MS; } while ((int32_t)(now - timing.next_connect_check) >= 0);
    // WiFi handling
    if (WiFi.status() != WL_CONNECTED) {
      Network::wifi_connect();
      if (WiFi.status() != WL_CONNECTED) {
        // Still down; suspend MQTT if not already
        if (!net_state.mqtt_suspended) {
          net_state.mqtt_suspended = true;
          Serial.println("MQTT suspended (WiFi down)");
        }
      }
    } else {
      // WiFi is up
      if (!mqtt.connected()) {
        if ((int32_t)(millis() - net_state.mqtt_cooldown_until) >= 0) {
          Network::mqtt_connect(mqtt);
          if (mqtt.connected()) {
            net_state.mqtt_suspended = false;
            net_state.mqtt_cooldown_until = 0;
            Serial.println("MQTT connected/resumed");
          } else {
            net_state.mqtt_cooldown_until = millis() + Config::MQTT_BACKOFF_MS;
            Serial.printf("MQTT connect failed; backing off %lus\n", (unsigned long)(Config::MQTT_BACKOFF_MS/1000));
          }
        }
      }
    }
    
    // Update network page UI lines
    IPAddress ip = WiFi.localIP();
    snprintf(ui_state.line_ip, sizeof(ui_state.line_ip), "IP:%s", 
             (WiFi.status() == WL_CONNECTED) ? ip.toString().c_str() : "0.0.0.0");
    Utils::pad16(ui_state.line_ip);
    
    const char* mq = (!net_state.mqtt_suspended && mqtt.connected()) ? "MQTT:OK" : 
                     (net_state.mqtt_suspended ? "MQTT:SUSP" : "MQTT:DOWN");
    strncpy(ui_state.line_netw, mq, sizeof(ui_state.line_netw));
    Utils::pad16(ui_state.line_netw);
    
    // Refresh network page if displayed
    if (ui_state.page == 3) Display::render(lcd, ui_state, net_state, mqtt);
  }

  if ((int32_t)(now - timing.next_rtc) >= 0) {
    if (timing.next_rtc == 0) timing.next_rtc = now; do { timing.next_rtc += Config::RTC_INTERVAL_MS; } while ((int32_t)(now - timing.next_rtc) >= 0);
    // Validate RTC running
    if (!Rtc.GetIsRunning()) {
      Rtc.SetIsRunning(true);
    }
    RtcDateTime rt = Rtc.GetDateTime();
    char buf[Config::UI_LINE_LENGTH];
    // Time HH:MM:SS
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", rt.Hour(), rt.Minute(), rt.Second());
    Utils::pad16(buf);
    strncpy(ui_state.line_time, buf, sizeof(ui_state.line_time));
    // Date DD/MM/YY
    snprintf(buf, sizeof(buf), "%02u/%02u/%04u", rt.Day(), rt.Month(), rt.Year());
    Utils::pad16(buf);
    strncpy(ui_state.line_date, buf, sizeof(ui_state.line_date));
    if (ui_state.page == 1) Display::render(lcd, ui_state, net_state, mqtt);
  }

  // MQTT publish every 5s aligned (only if not suspended)
  if (!net_state.mqtt_suspended && mqtt.connected() && (int32_t)(now - timing.next_mqtt) >= 0) {
    if (timing.next_mqtt == 0) timing.next_mqtt = now; do { timing.next_mqtt += Config::MQTT_INTERVAL_MS; } while ((int32_t)(now - timing.next_mqtt) >= 0);
    char payload[160];
    int tempOut = dht_state.have_last ? (int)roundf(dht_state.last_tC) : -99;
    int humOut = dht_state.have_last ? (int)roundf(dht_state.last_h) : -1;
    snprintf(payload, sizeof(payload),
             "{\"tempC\":%d,\"hum%%\":%d,\"dht_fail_consec\":%lu,\"ip\":\"%s\"}",
             tempOut, humOut, (unsigned long)dht_state.err_consec,
             WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString().c_str() : "0.0.0.0");
    mqtt.publish(Config::MQTT_TOPIC, payload);
    Serial.printf("MQTT publish: %s\n", payload);
  }

  // Service MQTT loop only if active
  if (!net_state.mqtt_suspended && mqtt.connected()) {
    mqtt.loop();
  }

  // LED policy:
  // - Red if connectivity failure (suspended or theft -> SOS blinking)
  // - Blue while connecting
  // - Else Green (idle/OK)
  if (security_state.theft_detected) {
    LED::handle_sos(sos_state, now);
  } else if (net_state.mqtt_suspended) {
    LED::red();
  } else if (net_state.mqtt_cooldown_until != 0 || WiFi.status() != WL_CONNECTED) {
    LED::blue();
  } else {
    LED::green();
  }

  // Button handling 
  static bool btn_armed = true; // one-shot: require full release before next advance
  bool btn = digitalRead(Pins::BTN);
  if (btn == LOW && btn_armed) { // pressed
    ui_state.page = (ui_state.page + 1) % Config::UI_PAGES; // wrap-around via modulo
    Display::render(lcd, ui_state, net_state, mqtt);
    btn_armed = false; // wait for release before next advance
  } else if (btn == HIGH) {
    btn_armed = true; // re-arm on release
  }

  // small idle pacing
  delay(10);
}