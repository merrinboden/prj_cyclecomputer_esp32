# CycleComputer

ESP32-based bike “cycle computer” demo showing live temperature/humidity, real-time clock, and motion activity on a 16x2 I2C LCD with a simple RGB status LED.

This project uses Arduino framework on ESP32 with common libraries via PlatformIO. It uses `LiquidCrystal_I2C` for the LCD, Adafruit `DHT` and `Adafruit Unified Sensor` for DHT11, `Adafruit MPU6050` for the accelerometer, and `makuna/RTC` for the DS1302 RTC.

## Features

- Temperature and humidity from `DHT11` every ~2 seconds
- Real-time clock via `DS1302` (3‑wire, via makuna/RTC), auto-set to firmware build time after upload
- Motion/change indication from `MPU6050` accelerometer (INT or 100 ms poll fallback)
- 16x2 LCD (I2C, address `0x27` by default) shows:
  - Line 1: `T:##C H:##%` or `DHT ERR <count>`
  - Line 2: `HH:MM:SSDD/MM/YY`
- RGB status LED policy:
  - Red: DHT failing (current cycle) or error count > 100
  - Blue: brief pulse while motion change is detected
  - Green: idle/OK
- One-time I2C scan at boot printed to Serial (115200 baud)
- Optional MQTT telemetry publish (WiFi + broker) every 5s

## Hardware

- ESP32 DevKit (env: `esp32dev`)
- DHT11 temperature/humidity sensor (data on GPIO 23)
- MPU6050 accelerometer (I2C at `0x68`, INT on GPIO 19)
- 16x2 LCD with PCF8574 backpack (I2C at `0x27`)
- DS1302 RTC, 3‑wire connection (RST/CE, DAT/IO, CLK)
- Common‑anode/cathode RGB LED with series resistors (active HIGH expected)

### Pinout (from `src/pins.hpp`)

- I2C `SDA`: `GPIO 21`
- I2C `SCL`: `GPIO 22`
- DHT11 `DATA`: `GPIO 23`
- MPU6050 `INT`: `GPIO 19`
- RGB LED: `R=GPIO 25`, `G=GPIO 26`, `B=GPIO 27` (active HIGH)
- DS1302: `RST=GPIO 18`, `DAT=GPIO 5`, `CLK=GPIO 4`

### I2C Addresses (from `src/pins.hpp`)

- LCD: `0x27`
- MPU6050: `0x68`

Notes:

- Supply and logic levels depend on your modules. Many PCF8574 LCD backpacks work at 5V but are I2C‑level tolerant via pull‑ups; ensure pull‑ups are safe for 3.3V ESP32, or use level shifting as needed.
- DHT11 supports 3.3V operation; follow its datasheet wiring and timing constraints.

## Build & Flash

This is a PlatformIO project using Arduino framework. Required libraries are declared in `platformio.ini` and will auto-install:

- `marcoschwartz/LiquidCrystal_I2C`
- `adafruit/DHT sensor library`
- `adafruit/Adafruit Unified Sensor`
- `adafruit/Adafruit MPU6050`
- `knolleary/PubSubClient` (MQTT client)
- `makuna/RTC` (DS1302 via ThreeWire)

### VS Code (recommended)

- Install the PlatformIO extension.
- Open this folder in VS Code.
- Environment: `env:esp32dev` (from `platformio.ini`).
- Connect your ESP32 (select the serial port if prompted).
- Build and Upload via the PlatformIO sidebar.
- Open Serial Monitor at `115200` baud to see logs and the I2C scan.

### PlatformIO CLI (PowerShell)

```powershell
pio run
pio run -t upload
pio device monitor -b 115200
```

## Behavior

- On first boot after each firmware upload, the RTC is set to the compile time. This is keyed by a stored “build signature” (`Preferences` NVS, key `app/buildSig`). Subsequent normal resets do not rewrite the time.
- LCD:
  - Line 1 shows DHT data or an incrementing error counter if reads fail.
  - Line 2 shows time and date in 16 chars: `HH:MM:SSDD/MM/YY`.
- LED: Red on DHT failure, Blue short pulse on motion events, Green when idle and healthy.
- Serial: I2C device scan results and motion event logs (when change detected).

## Configuration

- Pins and I2C addresses: `src/pins.hpp`.
- LCD address: `I2CAddr::LCD` in `src/pins.hpp` (default `0x27`).
- I2C clock: set in `src/main.cpp` (`Wire.setClock(100000)`).
- Motion sensitivity: change threshold and blue pulse window in `src/main.cpp`:
  - Threshold: `0.03f g`
  - Blue window: `200 ms`
- MQTT/WiFi: Edit constants (`WIFI_SSID`, `WIFI_PASS`, `MQTT_HOST`, `MQTT_PORT`, `MQTT_TOPIC`) near top of `src/main.cpp`.

## Project Structure

```text
src/
  main.cpp         # App logic (sensors, LCD, LED state machine, WiFi+MQTT)
  pins.hpp         # All pin assignments and I2C addresses
platformio.ini     # PlatformIO environment (esp32dev, Arduino)
```

## Troubleshooting

- LCD shows nothing or gibberish:
  - Verify the I2C address from Serial scan; update `I2CAddr::LCD` if needed.
  - Check contrast trimpot and supply voltage.
- DHT errors keep incrementing:
  - Check pull‑ups, wiring length, and that you read no more often than ~2s (the code does 2s).
  - Confirm the sensor is DHT11 (not DHT22) and on `GPIO 23`.
- No motion events / no Blue pulses:
  - Ensure MPU6050 wired to I2C with address `0x68` and INT on `GPIO 19`.
  - Check Serial for “MPU6050 OK/N/A” line; if N/A, check power and ground.
- RTC shows wrong time:
  - Re-upload firmware to set RTC to the new build time.
  - If battery-backed, ensure the cell is fresh.

## MQTT Setup

- Set WiFi and broker details in `src/main.cpp` near the top:
  - `WIFI_SSID`, `WIFI_PASS`
  - `MQTT_HOST`, `MQTT_PORT`, `MQTT_TOPIC`, `MQTT_CLIENT_ID`
- Build, upload, and open the serial monitor to confirm WiFi/MQTT connection.
- Verify messages by subscribing to the topic from your PC:

```powershell
# Using mosquitto (if installed)
mosquitto_sub -h <broker-host> -p 1883 -t cyclecomputer/telemetry -v
```

Notes:

- If your broker requires authentication, change the connect call in `main.cpp` from `mqtt.connect(MQTT_CLIENT_ID)` to `mqtt.connect(MQTT_CLIENT_ID, "USER", "PASS")`.
- Ensure your firewall allows outbound TCP to the broker’s port.

---
Made for coursework/experimentation; adapt pins and devices to your hardware.
