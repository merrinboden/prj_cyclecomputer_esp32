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
#include <stdint.h>

static LiquidCrystal_I2C lcd(I2CAddr::LCD, 16, 2);
static DHT dht(Pins::DHT, DHT11);
static Adafruit_MPU6050 mpu;
static bool mpu_ok = false;
static ThreeWire ds1302Bus(Pins::DS1302_DAT, Pins::DS1302_CLK, Pins::DS1302_RST);
static RtcDS1302<ThreeWire> Rtc(ds1302Bus);

// WiFi / MQTT configuration (set your credentials & broker)
static const char* WIFI_SSID = "Merrin's Pixel 8";
static const char* WIFI_PASS = "12345678";
static const char* MQTT_HOST = "10.220.204.1";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_CLIENT_ID = "cyclecomputer";
static const char* MQTT_TOPIC = "cyclecomputer/telemetry";
// Optional: set non-empty to use broker auth
static const char* MQTT_USER = ""; // e.g. "esp32"
static const char* MQTT_PASS = ""; // e.g. "s3cret"
static const uint32_t MQTT_BACKOFF_MS = 30000; // backoff after failed connect
static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);
// When WiFi is unavailable, suspend MQTT loop/publish attempts to avoid blocking retries
static bool mqtt_suspended = false;
static uint32_t mqtt_cooldown_until = 0;

// Event/state
static bool dht_ok = false;
static bool dht_failure = true;
static uint32_t dht_err_count = 0;
static uint32_t dht_fail_consec = 0;
static bool dht_have_last = false;
static float dht_last_tC = NAN, dht_last_h = NAN;
static float last_gx = 0, last_gy = 0, last_gz = 0;
static uint32_t mpu_change_until = 0; // keep LED blue until this time if change detected
// Aligned scheduling timestamps
static uint32_t next_dht = 0;
static uint32_t next_rtc = 0;
static uint32_t next_mpu = 0;
static uint32_t next_mqtt = 0;
static uint32_t next_connect_check = 0;
static uint32_t next_motion_page_refresh = 0; // throttles LCD refresh while on Motion page
// LCD pages
static uint8_t ui_page = 0; // 0: DHT, 1: Time, 2: Motion, 3: Network
static const uint8_t ui_pages = 4;
static char ui_line_dht[17] = "                ";
static char ui_line_time[17] = "                ";
static char ui_line_date[17] = "                ";
// DHT scheduling enhancements
static uint32_t next_dht_quick = 0;        // quick retry schedule after a failure
static uint32_t dht_warmup_until = 0;      // time until we start counting errors
static uint32_t dht_last_ok_ms = 0;        // millis of last successful reading
// Extended recovery state machine (opportunistic lightweight reads after a failure)
static uint32_t dht_recovery_until = 0;    // while now < this, perform recovery reads
static uint32_t next_dht_recovery = 0;     // next scheduled recovery attempt
static uint8_t dht_recovery_attempts = 0;  // attempts in current recovery window

// Inline LED helper (3-pin RGB, active HIGH per pins.hpp)
static void LedBegin(){
  pinMode(Pins::LED_R, OUTPUT);
  pinMode(Pins::LED_G, OUTPUT);
  pinMode(Pins::LED_B, OUTPUT);
  digitalWrite(Pins::LED_R, LOW);
  digitalWrite(Pins::LED_G, LOW);
  digitalWrite(Pins::LED_B, LOW);
}
static void LedSet(bool r,bool g,bool b){
  digitalWrite(Pins::LED_R, r?HIGH:LOW);
  digitalWrite(Pins::LED_G, g?HIGH:LOW);
  digitalWrite(Pins::LED_B, b?HIGH:LOW);
}
static void LedSetIdle(){ LedSet(false,true,false); }
static void LedForceRed(){ LedSet(true,false,false); }
static void LedForceBlue(){ LedSet(false,false,true); }
static void LedForceGreen(){ LedSetIdle(); }
static void LedUpdate(){}

static void pad16(char* s){ size_t l=strlen(s); while(l<16){ s[l++]=' '; } s[16]='\0'; }

static void render_display(){
  char l0[17]; char l1[17];
  if (ui_page == 0) {
    // DHT page
    strncpy(l0, ui_line_dht, 17);
    snprintf(l1, sizeof(l1), "DHT %s FC:%lu", dht_ok?"OK":"ERR", (unsigned long)dht_fail_consec);
    pad16(l1);
  } else if (ui_page == 1) {
    // Time page: time on upper line, date on lower line
    strncpy(l0, ui_line_time, 17);
    strncpy(l1, ui_line_date, 17);
  } else if (ui_page == 2) {
    // Motion page
    snprintf(l0, sizeof(l0), "g:%4.2f %4.2f", last_gx, last_gy);
    pad16(l0);
    snprintf(l1, sizeof(l1), "%4.2f %s", last_gz, "      ");
    pad16(l1);
  } else {
    // Network page
    IPAddress ip = WiFi.localIP();
    snprintf(l0, sizeof(l0), "IP:%s", (WiFi.status()==WL_CONNECTED)? ip.toString().c_str():"0.0.0.0");
    pad16(l0);
    const char* mq = (!mqtt_suspended && mqtt.connected())?"MQTT:OK": (mqtt_suspended?"MQTT:SUSP":"MQTT:DOWN");
    strncpy(l1, mq, sizeof(l1)); pad16(l1);
  }
  lcd.setCursor(0,0); lcd.print(l0);
  lcd.setCursor(0,1); lcd.print(l1);
}

static bool dht_read_retry(float &tC, float &hPct, int attempts=3, uint32_t gapMs=60) {
  for (int i=0;i<attempts;++i) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      tC = t; hPct = h; return true;
    }
    if (i+1 < attempts) delay(gapMs);
  }
  return false;
}

static void i2c_scan_once() {
  Serial.println("I2C scan...");
  for (uint8_t a=1;a<127;++a){ Wire.beginTransmission(a); if (Wire.endTransmission()==0) Serial.printf(" - 0x%02X\n", a);} }

// Non-blocking WiFi connect attempt (single begin call). Periodic status checked elsewhere.
static void wifi_connect() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("WiFi attempt: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// Non-blocking MQTT connect attempt (single try); retries scheduled by connectivity check.
static void mqtt_connect() {
  if (mqtt.connected()) return;
  Serial.printf("MQTT attempt: %s:%u\n", MQTT_HOST, MQTT_PORT);
  bool ok = false;
  if (MQTT_USER && *MQTT_USER) {
    ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
  } else {
    ok = mqtt.connect(MQTT_CLIENT_ID);
  }
  if (ok) {
    Serial.println("MQTT connected");
  } else {
    Serial.printf("MQTT connect failed rc=%d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200); delay(300);
  Wire.begin(Pins::SDA, Pins::SCL); Wire.setClock(100000);
  LedBegin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print("CycleComputer  ");
  lcd.setCursor(0,1); lcd.print("Init...         ");

  dht.begin();
  Rtc.Begin();
  dht_warmup_until = millis() + 10000; // 10s warm-up ignore error counts

  // Set RTC to compile time once per firmware upload using NVS build signature
  {
    Preferences prefs;
    prefs.begin("app", false);
    String buildSig = String(__DATE__) + " " + String(__TIME__);
    String prevSig = prefs.getString("buildSig", "");
    if (prevSig != buildSig) {
      RtcDateTime dt(__DATE__, __TIME__);
      Rtc.SetDateTime(dt);
      prefs.putString("buildSig", buildSig);
      Serial.println("RTC set to compile time (first boot after upload)");
    }
    prefs.end();
  }

  // MPU init (polling)
  mpu_ok = mpu.begin(I2CAddr::MPU);
  if (mpu_ok) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }
  Serial.printf("MPU6050 %s at 0x%02X\n", mpu_ok?"OK":"N/A", I2CAddr::MPU);

  // WiFi + MQTT init
  // Start asynchronous WiFi attempt; skip MQTT connect here to prevent blocking before sensors start.
  wifi_connect();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setSocketTimeout(1);   // seconds; keep connect() from blocking long
  mqtt.setKeepAlive(30);

  i2c_scan_once();
  LedSetIdle();
  // Button
  pinMode(Pins::BTN, INPUT_PULLUP);
}

void loop() {
  uint32_t now = millis();

  // DHT11 every 2500 ms (aligned, drift-free)
  if ((int32_t)(now - next_dht) >= 0) {
    if (next_dht == 0) next_dht = now; // initial immediate run
    // maintain alignment even if we overrun: add interval until next_dht in future
    do { next_dht += 2500; } while ((int32_t)(now - next_dht) >= 0);
    Serial.println("[loop] DHT tick");
    float h = NAN, t = NAN;
    bool ok = dht_read_retry(t, h, 2, 40); // reduce blocking; rely on recovery loop
    dht_ok = ok;
    if (!ok) {
      if (millis() >= dht_warmup_until) {
        dht_err_count++;
        dht_fail_consec++;
      }
      // schedule a quick retry in 600ms if none pending
      if (next_dht_quick == 0) next_dht_quick = millis() + 600;
      // start recovery window (3s) for opportunistic fast single reads
      if (dht_recovery_until < millis()) {
        dht_recovery_until = millis() + 3000;
        next_dht_recovery = millis() + 400; // first recovery attempt
        dht_recovery_attempts = 0;
      }
    } else {
      dht_err_count = 0;
      dht_fail_consec = 0;
      dht_failure = false;
      dht_last_tC = t; dht_last_h = h; dht_have_last = true;
      dht_last_ok_ms = millis();
      next_dht_quick = 0; // cancel quick retry if any
      dht_recovery_until = 0; next_dht_recovery = 0; dht_recovery_attempts = 0;
    }
    // Prepare DHT UI line
    if (ok) {
      snprintf(ui_line_dht, sizeof(ui_line_dht), "T:%dC H:%d%%", (int)roundf(t), (int)roundf(h));
    } else if (dht_have_last && dht_fail_consec < 3) {
      snprintf(ui_line_dht, sizeof(ui_line_dht), "T:%dC H:%d%%", (int)roundf(dht_last_tC), (int)roundf(dht_last_h));
    } else {
      // Show error or stale indicator
      uint32_t age = dht_last_ok_ms ? (millis() - dht_last_ok_ms) : 0;
      if (dht_have_last && age > 30000) {
        snprintf(ui_line_dht, sizeof(ui_line_dht), "STALE %lu", (unsigned long)(age/1000));
      } else {
        snprintf(ui_line_dht, sizeof(ui_line_dht), "DHT ERR %lu", (unsigned long)dht_err_count);
      }
    }
    pad16(ui_line_dht);
    if (ui_page == 0) render_display();
  }

  // Quick retry logic (non-aligned, opportunistic) executes only if scheduled and after warm-up
  if (next_dht_quick != 0 && (int32_t)(now - next_dht_quick) >= 0) {
    // perform a lightweight single attempt without blocking other tasks
    next_dht_quick = 0; // clear schedule first to avoid reentrancy
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    bool ok = !isnan(h) && !isnan(t);
    if (ok) {
      dht_ok = true;
      dht_last_tC = t; dht_last_h = h; dht_have_last = true; dht_last_ok_ms = millis();
      dht_err_count = 0; dht_fail_consec = 0; dht_failure = false;
      snprintf(ui_line_dht, sizeof(ui_line_dht), "T:%dC H:%d%%", (int)roundf(t), (int)roundf(h));
      pad16(ui_line_dht);
      if (ui_page == 0) render_display();
    } else {
      // schedule another quick retry only if we haven't exceeded a short burst (max 2 quick retries between main intervals)
      static uint8_t quick_retry_count = 0;
      if (quick_retry_count < 2) {
        next_dht_quick = millis() + 700;
        quick_retry_count++;
      } else {
        quick_retry_count = 0; // reset for next main interval
      }
    }
  }

  // Recovery state machine: sparse single reads for a short window after a failure
  if (dht_recovery_until && millis() < dht_recovery_until && next_dht_recovery && (int32_t)(now - next_dht_recovery) >= 0) {
    next_dht_recovery = 0; // clear before attempt
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    bool ok = !isnan(h) && !isnan(t);
    if (ok) {
      dht_ok = true; dht_last_tC = t; dht_last_h = h; dht_have_last = true; dht_last_ok_ms = millis();
      dht_err_count = 0; dht_fail_consec = 0; dht_failure = false;
      snprintf(ui_line_dht, sizeof(ui_line_dht), "T:%dC H:%d%%", (int)roundf(t), (int)roundf(h)); pad16(ui_line_dht);
      if (ui_page == 0) render_display();
      dht_recovery_until = 0; dht_recovery_attempts = 0; // end recovery
    } else {
      dht_recovery_attempts++;
      if (dht_recovery_attempts < 5) {
        next_dht_recovery = millis() + 450; // schedule another quick attempt
      } else {
        // stop recovery; wait for next aligned interval
        dht_recovery_until = 0; dht_recovery_attempts = 0;
      }
    }
  }

  // MPU read every 100ms aligned
  bool mpu_event=false;
  if (mpu_ok && (int32_t)(now - next_mpu) >= 0) {
    if (next_mpu == 0) next_mpu = now; do { next_mpu += 100; } while ((int32_t)(now - next_mpu) >= 0);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float gx = a.acceleration.x / 9.80665f;
    float gy = a.acceleration.y / 9.80665f;
    float gz = a.acceleration.z / 9.80665f;
    if (fabsf(gx-last_gx)>0.03f || fabsf(gy-last_gy)>0.03f || fabsf(gz-last_gz)>0.03f) {
      mpu_event = true; mpu_change_until = now + 200; // blue LED window
      last_gx=gx; last_gy=gy; last_gz=gz;
      Serial.printf("MPU g: %.2f, %.2f, %.2f\n", gx, gy, gz);
    }
    // Refresh motion page either on detected motion change or every 500ms while displayed
    if (ui_page == 2 && (mpu_event || (int32_t)(now - next_motion_page_refresh) >= 0)) {
      next_motion_page_refresh = now + 500; // throttle updates
      render_display();
    }
  }

  // Connectivity periodic check (avoid constant calls causing jitter)
  if ((int32_t)(now - next_connect_check) >= 0) {
    if (next_connect_check == 0) next_connect_check = now; do { next_connect_check += 5000; } while ((int32_t)(now - next_connect_check) >= 0);
    // WiFi handling
    if (WiFi.status() != WL_CONNECTED) {
      wifi_connect();
      if (WiFi.status() != WL_CONNECTED) {
        // Still down; suspend MQTT if not already
        if (!mqtt_suspended) {
          mqtt_suspended = true;
          Serial.println("MQTT suspended (WiFi down)");
        }
      }
    } else {
      // WiFi is up
      if (!mqtt.connected()) {
        if ((int32_t)(millis() - mqtt_cooldown_until) >= 0) {
          mqtt_connect();
          if (mqtt.connected()) {
            mqtt_suspended = false;
            mqtt_cooldown_until = 0;
            Serial.println("MQTT connected/resumed");
          } else {
            mqtt_cooldown_until = millis() + MQTT_BACKOFF_MS;
            Serial.printf("MQTT connect failed; backing off %lus\n", (unsigned long)(MQTT_BACKOFF_MS/1000));
          }
        } else {
          // In cooldown; skip connect attempt
        }
      }
    }
  }

  // RTC date+time to LCD line 1 once per second (HH:MM:SSDD/MM/YY fits 16 chars)
  if ((int32_t)(now - next_rtc) >= 0) {
    if (next_rtc == 0) next_rtc = now; do { next_rtc += 1000; } while ((int32_t)(now - next_rtc) >= 0);
    // Validate RTC running
    if (!Rtc.GetIsRunning()) {
      Serial.println("RTC not running; starting it.");
      Rtc.SetIsRunning(true);
    }
    RtcDateTime rt = Rtc.GetDateTime();
    Serial.println("[loop] RTC tick");
    char buf[17];
    uint8_t yy = (uint8_t)(rt.Year() % 100);
    // Time HH:MM:SS
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", rt.Hour(), rt.Minute(), rt.Second());
    pad16(buf);
    strncpy(ui_line_time, buf, sizeof(ui_line_time));
    // Date DD/MM/YY
    snprintf(buf, sizeof(buf), "%02u/%02u/%02u", rt.Day(), rt.Month(), yy);
    pad16(buf);
    strncpy(ui_line_date, buf, sizeof(ui_line_date));
    if (ui_page == 1) render_display();
  }
  // MQTT publish every 5s aligned (only if not suspended)
  if (!mqtt_suspended && mqtt.connected() && (int32_t)(now - next_mqtt) >= 0) {
    if (next_mqtt == 0) next_mqtt = now; do { next_mqtt += 5000; } while ((int32_t)(now - next_mqtt) >= 0);
    bool motion = (int32_t)(now - mpu_change_until) < 0;
    char payload[160];
    int tempOut = dht_have_last ? (int)roundf(dht_last_tC) : -99;
    int humOut = dht_have_last ? (int)roundf(dht_last_h) : -1;
    snprintf(payload, sizeof(payload),
             "{\"tempC\":%d,\"hum%%\":%d,\"motion\":%s,\"dht_fail_consec\":%lu,\"ip\":\"%s\"}",
             tempOut, humOut, motion?"true":"false", (unsigned long)dht_fail_consec,
             WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString().c_str() : "0.0.0.0");
    mqtt.publish(MQTT_TOPIC, payload);
    Serial.printf("MQTT publish: %s\n", payload);
  }

  // Service MQTT loop (non-blocking) only if active
  if (!mqtt_suspended && mqtt.connected()) {
    mqtt.loop();
  }

  // LED policy:
  // - Red if any failure (DHT read fail this cycle) or dht_err_count > 100
  // - Else Blue while MPU changes (until mpu_change_until)
  // - Else Green (idle)
  bool showRed = dht_failure || (dht_err_count > 100);
  if (showRed) {
    LedForceRed();
  } else if ((int32_t)(now - mpu_change_until) < 0) {
    LedForceBlue();
  } else {
    LedForceGreen();
  }
  LedUpdate();

  // Button handling (debounced, on press cycle pages)
  static bool btn_last = true; // pull-up idle HIGH
  static uint32_t btn_last_change = 0;
  static bool btn_armed = true; // one-shot: require full release before next advance
  bool btn = digitalRead(Pins::BTN);
  if (btn != btn_last) {
    if (millis() - btn_last_change > 30) { // debounce both edges (shortened)
      btn_last = btn;
      btn_last_change = millis();
      if (btn == LOW && btn_armed) { // pressed
        ui_page = (ui_page + 1) % ui_pages; // wrap-around via modulo
        render_display();
        btn_armed = false; // wait for release before next advance
      } else if (btn == HIGH) {
        btn_armed = true; // re-arm on release
      }
    }
  }

  // small idle pacing
  delay(10);
}