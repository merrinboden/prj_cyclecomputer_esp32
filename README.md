# ESP32 Fahrrad-Computer / Cycle Computer

Projektziel: Entwicklung eines stromsparenden Fahrrad-Computers auf ESP32-Basis mit Telemetrie, Anzeige und Diebstahlschutz.

## Überblick

- Vereinfachte Zustandsmaschine (INIT, IDLE, ACTIVE, DISCONNECTED) für klare Ablaufsteuerung
- CoAP-Server-Architektur: ESP32 hostet Ressourcen, Android-App pollt Telemetriedaten
- 16x2 I2C-LCD zur Anzeige von Uhrzeit, Umwelt- und Bewegungsdaten; einfache UI-Pages per Button
- Energieoptimierung durch adaptive Sensor-Abtastrate und Loop-Verzögerungen

## Hardware

- ESP32 DevKit (Arduino-Framework)
- Sensoren: DHT22 (Temperatur/Luftfeuchte), MPU6050 (Beschleunigung/Gyro)
- Uhr: DS1302 RTC (Zeit für Anzeige/Logs; Set auf Build-Zeit beim Flashen)
- Anzeige: 16x2 I2C LCD (`LiquidCrystal_I2C`)
- LED: RGB-LED für Statusmeldungen (Rot/Blau/Grün, SOS-Pattern bei Diebstahl)
- Button: Seitenwechsel und Backlight-Aktivierung; konfigurierbar per `Config::BUTTON_PULLUP` (true=Pull-up, false=Pull-down)
  - Erste Taste nach LCD-Timeout: Nur Backlight aktivieren, keine Seitenwechsel
  - Nachfolgende Tasten: Normale Seitenwechsel-Funktion

## Software & Bibliotheken

- PlatformIO-Projekt mit Arduino-Core
- Bibliotheken: DHT_Unified, Adafruit_MPU6050, LiquidCrystal_I2C, RtcDS1302, CoAP-simple, WiFi/WiFiUDP
- Projektstruktur: `src/` (Logik), `lib/` (ggf. Zusatz), `test/` (Platzhalter)

## Zustände & Logik

- **INIT**: Hardware-Init, erste Sensorchecks, WLAN-Start, CoAP-Server-Initialisierung
- **IDLE**: Verbunden, keine Bewegung; Sensorfrequenz 1 Hz (1s Intervall); Server bereit für Anfragen
- **ACTIVE**: Verbunden, Bewegung erkannt; Sensorfrequenz 5 Hz (200ms Intervall); Server bereit für Anfragen
- **DISCONNECTED**: WLAN verloren; Reconnect-Versuch alle 10 Minuten; Display bleibt funktional

## Sensorik

### DHT22

- Gelesen mit `Config::DHT_READ_MS`-Throttling (2000ms); Initialcheck im Setup (OK/FAILED)
- Fallback-Werte bei Ausfall: 20.0°C Temperatur, 50% Luftfeuchtigkeit

### MPU6050

- Bewegungsdetektion über Magnituden von Beschleunigung und Gyro; Schwellwerte konfigurierbar
- Achsen-Mapping: Z→X, Y→Y, X→Z (für korrekte Orientierung am Fahrrad)
- Fallback-Werte bei Ausfall: ax=0.0, ay=0.0, az=9.81, gx/gy/gz=0.0

### Bewegungserkennung

- `MOVEMENT_ACCEL_THRESHOLD` (1.0 m/s²), `MOVEMENT_GYRO_THRESHOLD` (5.0 rad/s)
- Timeout `MOVEMENT_TIMEOUT_MS` (10s)

### Diebstahlschutz

- Verwendet gleiche Bewegungserkennung wie normale Bewegung; nur aktiv wenn `state.locked == true`
- LOCK-Befehl aktiviert Überwachung, UNLOCK deaktiviert und setzt Alarm zurück
- Bei Bewegung im gesperrten Zustand: `theft_detected = true` → LED-SOS-Pattern

## Kommunikation (CoAP-Server)

ESP32 agiert als CoAP-Server auf Port 5683.

### Ressourcen

**`/telemetry` (GET)**: Liefert aktuelle Sensordaten als JSON

```json
{
  "t": 24.5,       // Temperatur (Celsius)
  "h": 55.2,       // Luftfeuchtigkeit (%)
  "ax": 0.12,      // Beschleunigung X (m/s²)
  "ay": -0.05,     // Beschleunigung Y (m/s²)
  "az": 0.98,      // Beschleunigung Z (m/s²)
  "gx": 0.01,      // Gyro X (rad/s)
  "gy": 0.02,      // Gyro Y (rad/s)
  "gz": -0.01,     // Gyro Z (rad/s)
  "theft": "N",    // Diebstahlstatus: "Y" = erkannt, "N" = nicht erkannt
  "s": "A"         // Status: "A" für Active, "I" für Idle
}
```

**`/cmd` (POST)**: Empfängt Befehle (z.B. "LOCK", "UNLOCK")

- **LOCK**: Aktiviert Diebstahlüberwachung (`state.locked = true`)
- **UNLOCK**: Deaktiviert Überwachung (`state.locked = false`) und setzt Diebstahlalarm zurück

### Android-App Integration

- Android-App pollt Daten nach Bedarf (empfohlen: 1-2s Intervall)
- CoAP-Loop läuft mit 100ms Intervall für responsive Serverantworten
- Re-Init von CoAP-Server nach WLAN-Reconnect

## Energie & Power-Management

- WiFi-Light-Sleep ist **deaktiviert** (`disableWiFiSleep()`) für zuverlässige CoAP-Server-Verfügbarkeit
- Adaptive Loop-Verzögerungen: Active=20ms, Idle=100ms, Disconnected=1000ms
- Adaptive Sensor-Abtastrate: Active=200ms (5Hz), Idle=1000ms (1Hz)
- Kein Deep-Sleep im Server-Modus (für permanente Verfügbarkeit)

### Stromverbrauch & Batterielaufzeit-Schätzungen

**Komponentenverbrauch (typische Werte)**:

- ESP32 (240 MHz, WiFi aktiv, kein Sleep): ~160-240 mA
- ESP32 (240 MHz, WiFi aktiv, Light Sleep): ~20-30 mA (nicht verwendet)
- WiFi TX/RX Spitzen: bis 400 mA kurzzeitig
- LCD mit Backlight: ~20-30 mA
- LCD ohne Backlight: ~5 mA
- DHT22: ~1.5 mA (während Messung), <0.1 mA (Standby)
- MPU6050: ~3.5 mA (aktiv), ~8 µA (Sleep, nicht verwendet)
- DS1302 RTC: ~300 µA
- RGB LED (eine Farbe): ~5-20 mA (je nach Helligkeit)

**Durchschnittlicher Verbrauch nach Zustand**:

1. **IDLE** (WiFi verbunden, LCD Backlight aus, Sensoren 1 Hz): ~197 mA
2. **ACTIVE** (WiFi verbunden, LCD Backlight an, Sensoren 5 Hz): ~239 mA
3. **DISCONNECTED** (WiFi Reconnect-Versuche, LCD Backlight aus): ~165 mA
4. **THEFT_ALERT** (gesperrt, Bewegung erkannt): ~212 mA

**Geschätzte Batterielaufzeit**:

| Akku-Kapazität | Normale Fahrt (220 mA) | Überwachung (181 mA) | Diebstahlalarm (212 mA) |
|----------------|------------------------|----------------------|-------------------------|
| 1000 mAh       | 4,5 h                  | 5,5 h                | 4,7 h                   |
| 2500 mAh       | 11,4 h                 | 13,8 h               | 11,8 h                  |
| 5000 mAh       | 22,7 h                 | 27,6 h               | 23,6 h                  |
| 10000 mAh      | 45,5 h                 | 55,2 h               | 47,2 h                  |

**Hinweise**:

- Reale Laufzeit liegt ca. 80-85% der berechneten Werte (Batterie-Effizienz, Spannungswandler-Verluste)
- CoAP-Polling-Frequenz der Android-App beeinflusst WiFi-Aktivität und Verbrauch
- LCD-Backlight-Nutzung hat signifikanten Einfluss (~25 mA Unterschied)

**Optimierungsempfehlungen**:

- LCD-Backlight möglichst kurz nutzen (30s Timeout ist sinnvoll)
- Bei Nichtnutzung: System in DISCONNECTED-Modus versetzen (WiFi aus)
- Für längere Überwachung: Externe Stromversorgung (USB) empfohlen
- Zukünftig: WiFi-Light-Sleep implementieren für Überwachungsmodus (kann Verbrauch auf ~50-80 mA senken)

## Anzeige & LED

### LCD-Pages

- Zeit/Datum (RTC)
- Umwelt (Temperatur, Luftfeuchte + DHT-Status)
- Bewegung (Accel/Gyro live)
- Netzwerk/State (WiFi-Status, lokale IP)

### LCD-Hintergrundbeleuchtung

- Automatisches Timeout nach 30 Sekunden Inaktivität
- Reaktivierung durch Tastendruck
- Sofortige Aktivierung bei Benutzereingabe für direktes Feedback

### LED-Status (Priorität absteigend)

- **THEFT_ALERT**: Rot-SOS (3 kurz, 3 lang, 3 kurz) - höchste Priorität, nur wenn gesperrt und Diebstahl erkannt
- **COAP_ERROR**: Rot - wenn WiFi verbunden aber keine CoAP-Aktivität seit >30s
- **INIT**: Blau-Blink - während Initialisierung oder WiFi-Verbindung
- **SYSTEM_OK**: Grün - normaler Betrieb, alle Systeme funktional

## Netzwerkdiagnostik

- WLAN-Scan vor Verbindungsaufbau; klare Statuslogs (z. B. `WL_NO_SSID_AVAIL`)
- Auto-Reconnect: Beim Boot deaktiviert für kontrollierte Verbindungsversuche; nach erfolgreicher Verbindung automatisch aktiviert
- Manuelle Reconnect-Versuche: alle 60 Sekunden wenn nicht verbunden
- UDP-Socket wird bei Disconnect sofort gestoppt und bei Reconnect vollständig neu initialisiert
- CoAP-Server erst nach erfolgreichem WLAN-Connect gestartet

## Build & Deployment

**PlatformIO**: Build, Upload, serielle Logs für Diagnose

### VS Code (empfohlen)

```bash
# PlatformIO Extension installieren, dann:
# Build: PlatformIO: Build
# Upload: PlatformIO: Upload
# Monitor: PlatformIO: Serial Monitor (115200 baud)
```

### PlatformIO CLI (PowerShell)

```powershell
pio run
pio run -t upload
pio device monitor -b 115200
```

**RTC-Initialisierung**: RTC wird beim Flashen auf Build-Zeit gesetzt (aus `__DATE__`/`__TIME__`)

## Fehlerbehebung (Troubleshooting)

- **WLAN verbindet nicht**: SSID im 2,4 GHz-Band, nicht verborgen; Gerät näher am AP; SSID exakt prüfen
- **DHT unzuverlässig**: Stabilisierung (200 ms), Lese-Intervall erhöhen, Verkabelung/5V/3,3V und Pullup prüfen
- **Button reagiert nicht**: Pull-up/-down-Config beachten, Rohwerte loggen, Verdrahtung kontrollieren
- **UDP-Fehler-Spam**: CoAP-Loop nur bei WLAN-Verbindung ausführen

## Roadmap

- Exponentielles Backoff für WiFi-Reconnect (aktuell fest 60s)
- Filterung/Glättung von Bewegungsdaten für stabilere Erkennung
- Batteriespannungsüberwachung und Warnung bei niedrigem Ladestand
- WiFi-Light-Sleep für Überwachungsmodus implementieren

## Systemarchitektur

Der ESP32 Fahrrad-Computer ist ein autonomes Edge-Computing-System, das speziell für den Einsatz an Fahrrädern entwickelt wurde. Es fungiert als intelligentes IoT-Gerät, das Umgebungsdaten und Bewegungsaktivitäten in Echtzeit erfasst, lokal verarbeitet und über drahtlose Kommunikationstechnologien an einen zentralen Server übermittelt.

### Hardware-Architektur

Das Herzstück des Systems bildet ein ESP32 DevKit mit einem Dual-Core Xtensa LX6 Prozessor, der mit 240 MHz getaktet ist. Die Sensorik besteht aus einem DHT22-Sensor (Temperatur/Luftfeuchtigkeit) und einem MPU6050 (6-Achsen IMU). Eine DS1302 Echtzeituhr (RTC) mit Batterie-Pufferung gewährleistet kontinuierliche Zeitmessung. Die Benutzerinteraktion erfolgt über ein 16x2 LCD-Display (I2C), eine RGB-LED (Statusanzeige) und einen Drucktaster.

### Software-Architektur

Die Software basiert auf dem Arduino-Framework und wird mittels PlatformIO entwickelt. Die Anwendungslogik ist als vereinfachte Finite State Machine implementiert mit vier definierten Zuständen:

- **INIT**: Hardware-Initialisierung, Sensorchecks, WiFi-Verbindung, CoAP-Server-Start
- **IDLE**: Energiesparmodus mit reduzierter Sensor-Abtastrate (1 Hz) bei stehendem Fahrrad
- **ACTIVE**: Aktiver Modus mit erhöhter Sensor-Abtastrate (5 Hz) bei Bewegung
- **DISCONNECTED**: WiFi-Reconnect-Versuche bei verlorenem Netzwerk

### Kommunikation und Datenprotokoll

Das System nutzt das CoAP-Protokoll (Constrained Application Protocol) über UDP, ideal für IoT-Anwendungen durch geringen Overhead. Das Gerät agiert als CoAP-Server und hostet Ressourcen, die von einer Android-App abgefragt werden können.

Die Nutzdaten werden als kompakte JSON-Objekte formatiert und enthalten vollständige Informationen: Temperatur, Luftfeuchtigkeit sowie Rohdaten der Beschleunigungs- und Gyroskopsensoren aller drei Achsen, Diebstahlstatus und Gerätestatus.

### Energie-Management

Das System nutzt adaptive Verzögerungen in der Hauptschleife (Loop), die je nach Systemzustand variieren: 20ms im ACTIVE-Zustand für schnelle Reaktion, 100ms im IDLE-Zustand für Energieeinsparung, und 1000ms im DISCONNECTED-Zustand zur Minimierung des Verbrauchs. Das WiFi-Modul bleibt permanent aktiv (Light-Sleep deaktiviert), um als zuverlässiger CoAP-Server zu fungieren. Deep-Sleep ist bewusst nicht implementiert, da das System kontinuierlich für Abfragen der Android-App verfügbar sein muss.

## Android-Client-Implementierung

### Projektdokumentation: App zum CycleComputer

**Version**: 1.0 | **Datum**: 11. Dezember 2025

#### 1. Projektübersicht

Das Projekt "CycleComputer" ist eine native Android-Anwendung, die als umfassender Fahrradcomputer dient. Die App verbindet sich über das CoAP-Protokoll mit einem externen IoT-Gerät (z.B. einem ESP32-Mikrocontroller), um Echtzeit-Telemetriedaten zu empfangen und zu visualisieren. Zu den erfassten Daten gehören Temperatur, Luftfeuchtigkeit, Beschleunigungs- und Gyroskopwerte sowie ein Diebstahl- und Gerätestatus. Benutzer können Befehle an das Gerät senden, wie das Ver- oder Entriegeln eines digitalen Schlosses. Die App integriert außerdem Standortdienste, um die aktuelle Geschwindigkeit zu berechnen und detaillierte Wetterdaten für den aktuellen Standort abzurufen.

#### 2. Technische Architektur

Die Anwendung folgt einer Single-Activity-Architektur (DataActivity.java) und nutzt moderne Android-Komponenten und Bibliotheken:

- **Kommunikationsprotokoll**: CoAP (californium-core-Bibliothek)
- **Datenverarbeitung**: JSON-Parsing mit Gson-Bibliothek in stark typisierte Java-Objekte (TelemetryData)
- **UI-Aktualisierung**: Android Architecture Components (LiveData und Observer) für reaktive Benutzeroberfläche
- **Standort und Geschwindigkeit**: FusedLocationProviderClient für Standort; Geschwindigkeitsberechnung durch Integration von Beschleunigungssensordaten mit Hochpassfilter
- **Netzwerkanfragen**: Fuel-Bibliothek für Wetterdaten von weatherapi.com-API

#### 3. Kernkomponenten

**a) DataActivity.java**: Zentrale Activity für UI-Management, Datenvisualisierung, Benutzerinteraktionen, Lifecycle-Management und Berechtigungsmanagement

**b) CoapManager.java (Singleton)**: Kapselt CoAP-Kommunikationslogik, Verbindungsmanagement ("observe"-Beziehung), Datenbereitstellung über LiveData, Status-Management und Befehle senden (PUT-Anfragen)

**c) TelemetryData.java**: POJO-Datenklasse mit Feld für Diebstahlwarnung (theft)

**d) WeatherApiResponse.java**: Verschachtelte Datenklasse für JSON-Antwort der Wetter-API

### App-Dokumentation (Benutzerhandbuch)

#### Hauptbildschirm & Statusanzeige

Die Hintergrundfarbe signalisiert den Verbindungsstatus:

- **Grau**: App bereit, keine aktiven Daten
- **Grün**: Aktive Verbindung, Daten werden empfangen
- **Rot**: Verbindung unterbrochen

#### Angezeigte Daten

- **DIEBSTAHL!**: Auffälliger roter Text bei Diebstahlaktivität (theft = "Y")
- **Gerätestatus**: Betriebsstatus (z.B. "ACTIVE" oder "IDLE")
- **Geschwindigkeit**: Aktuelle Fahrgeschwindigkeit in km/h
- **Wetter**: Detaillierte Wetterinformationen (Beschreibung, Temperatur, gefühlte Temperatur, Luftfeuchtigkeit, Wind)
- **Temperatur**: Umgebungstemperatur in °C
- **Luftfeuchtigkeit**: Relative Luftfeuchtigkeit in %

#### Steuerelemente

- **Sperren/Entsperren (Schalter)**: Wählt gewünschten Zustand des digitalen Schlosses
- **SENDE BEFEHL (Button)**: Sendet "LOCK" oder "UNLOCK" Befehl an Fahrrad
- **ROHDATEN ANZEIGEN/VERBERGEN (Button)**: Blendet detaillierte Sensordaten ein/aus (Beschleunigung X/Y/Z, Gyroskop X/Y/Z)

#### Berechtigungen

Beim ersten Start wird Standortzugriff (ACCESS_FINE_LOCATION) benötigt für präzise Wetterdaten. Bei Verweigerung erscheint "N/A" als Wetter.

---

**Hinweise**:

- APIs: Optional geplante Nutzung externer Wetter-/Zeit-Dienste; aktuell lokal über RTC
- Zielkriterium: stabiles, energieeffizientes Laufverhalten (>1 h) mit verlässlicher Anzeige/Telemetrie
- Made for coursework/experimentation; adapt to your hardware
