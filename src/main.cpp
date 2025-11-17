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
static const char* WIFI_SSID = "YOUR_SSID";
static const char* WIFI_PASS = "YOUR_PASSWORD";
static const char* MQTT_HOST = "broker.example.com";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_CLIENT_ID = "cyclecomputer";
static const char* MQTT_TOPIC = "cyclecomputer/telemetry";
static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);

// Event/state
static bool dht_ok = false;
static bool dht_failure = true;
static uint32_t dht_err_count = 0;
static uint32_t dht_fail_consec = 0;
static bool dht_have_last = false;
static float dht_last_tC = NAN, dht_last_h = NAN;
static float last_gx = 0, last_gy = 0, last_gz = 0;
static uint32_t mpu_change_until = 0; // keep LED blue until this time if change detected

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

static void wifi_connect() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("WiFi connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi connect failed (timeout)");
  }
}

static void mqtt_connect() {
  if (mqtt.connected()) return;
  Serial.printf("MQTT connecting to %s:%u...\n", MQTT_HOST, MQTT_PORT);
  uint8_t attempts = 0;
  while (!mqtt.connected() && attempts < 5) {
    if (mqtt.connect(MQTT_CLIENT_ID)) {
      Serial.println("MQTT connected");
      break;
    } else {
      Serial.printf("MQTT connect failed rc=%d; retrying...\n", mqtt.state());
      attempts++;
      delay(2000);
    }
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
  wifi_connect();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt_connect();

  i2c_scan_once();
  LedSetIdle();
}

void loop() {
  static uint32_t t_dht = 0, t_rtc = 0, t_idle = 0, t_pub = 0;
  
  uint32_t now = millis();

  // DHT11 every 2500 ms (spec min ~2s; a bit more can help stability)
  if (now - t_dht >= 2500) {
    t_dht = now;
    float h = NAN, t = NAN;
    bool ok = dht_read_retry(t, h, 2, 80);
    dht_ok = ok;
    if (!ok) {
      dht_err_count++;
      dht_fail_consec++;
    } else {
      dht_err_count = 0;
      dht_fail_consec = 0;
      dht_failure = false;
      dht_last_tC = t; dht_last_h = h; dht_have_last = true;
    }
    char line0[17];
    if (ok) {
      snprintf(line0, sizeof(line0), "T:%dC H:%d%%", (int)roundf(t), (int)roundf(h));
    } else if (dht_have_last && dht_fail_consec < 3) {
      // Show last known good reading for up to 2 consecutive failures
      snprintf(line0, sizeof(line0), "T:%dC H:%d%%", (int)roundf(dht_last_tC), (int)roundf(dht_last_h));
    } else {
      snprintf(line0, sizeof(line0), "DHT ERR %lu", (unsigned long)dht_err_count);
    }
    // Pad to 16
    int l=strlen(line0); while(l<16) line0[l++]=' '; line0[16]='\0';
    lcd.setCursor(0,0); lcd.print(line0);
  }

  // MPU read at 100ms
  static uint32_t t_mpu = 0; bool mpu_event=false;
  if (mpu_ok && (now - t_mpu >= 100)) {
    t_mpu = now;
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
  }

  // RTC date+time to LCD line 1 once per second (HH:MM:SSDD/MM/YY fits 16 chars)
  if (now - t_rtc >= 1000) {
    t_rtc = now; RtcDateTime rt = Rtc.GetDateTime();
    char line1[17];
    uint8_t yy = (uint8_t)(rt.Year() % 100);
    snprintf(line1, sizeof(line1), "%02u:%02u:%02u%02u/%02u/%02u", rt.Hour(), rt.Minute(), rt.Second(), rt.Day(), rt.Month(), yy);
    // Ensure null termination and full width
    line1[16] = '\0';
    lcd.setCursor(0,1); lcd.print(line1);
  }
  // MQTT publish every 5s if connected
  if (mqtt.connected() && (now - t_pub >= 5000)) {
    t_pub = now;
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

  // Service MQTT
  if (WiFi.status() != WL_CONNECTED) wifi_connect();
  if (!mqtt.connected()) mqtt_connect();
  mqtt.loop();

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

  // small idle pacing
  delay(10);
}