#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "pins.hpp"
#include "ds1302.hpp"
#include "status_led.hpp"

static LiquidCrystal_I2C lcd(I2CAddr::LCD, 16, 2);
static DHT dht(Pins::DHT, DHT11);
static Adafruit_MPU6050 mpu;
static bool mpu_ok = false;

// Event/state
static bool dht_ok = false;
static bool dht_failure = true;
static uint32_t dht_err_count = 0;
static uint32_t dht_fail_consec = 0;
static bool dht_have_last = false;
static float dht_last_tC = NAN, dht_last_h = NAN;
static float last_gx = 0, last_gy = 0, last_gz = 0;
static uint32_t mpu_change_until = 0; // keep LED blue until this time if change detected

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

void setup() {
  Serial.begin(115200); delay(300);
  Wire.begin(Pins::SDA, Pins::SCL); Wire.setClock(100000);
  StatusLed::begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print("CycleComputer  ");
  lcd.setCursor(0,1); lcd.print("Init...         ");

  dht.begin();
  DS1302::begin();

  // Set RTC to compile time once per firmware upload using NVS build signature
  {
    Preferences prefs;
    prefs.begin("app", false);
    String buildSig = String(__DATE__) + " " + String(__TIME__);
    String prevSig = prefs.getString("buildSig", "");
    if (prevSig != buildSig) {
      DS1302::setCompileTime();
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

  i2c_scan_once();
  StatusLed::setIdle();
}

void loop() {
  static uint32_t t_dht = 0, t_rtc = 0, t_idle = 0;
  
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
    t_rtc = now; RtcTime rt = DS1302::readTime();
    char line1[17];
    // Format without space between time and date to fit year in 16 columns
    snprintf(line1, sizeof(line1), "%02u:%02u:%02u%02u/%02u/%02u", rt.hour, rt.min, rt.sec, rt.date, rt.month, rt.year);
    // Ensure null termination and full width
    line1[16] = '\0';
    lcd.setCursor(0,1); lcd.print(line1);
  }

  // LED policy:
  // - Red if any failure (DHT read fail this cycle) or dht_err_count > 100
  // - Else Blue while MPU changes (until mpu_change_until)
  // - Else Green (idle)
  bool showRed = dht_failure || (dht_err_count > 100);
  if (showRed) {
    StatusLed::forceRed();
  } else if ((int32_t)(now - mpu_change_until) < 0) {
    StatusLed::forceBlue();
  } else {
    StatusLed::forceGreen();
  }
  // Allow any pending blue pulse to auto-clear
  StatusLed::update();

  // small idle pacing
  delay(10);
}