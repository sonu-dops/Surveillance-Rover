# 🛰️ Surveillance Rover

A multi-microcontroller surveillance rover with real-time sensor monitoring, GPS tracking, camera gimbal stabilization, and remote control capability.

---

## 📐 System Architecture

```
┌─────────────────────┐        Serial (UART)        ┌─────────────────────┐
│   Arduino Uno R3    │◄───────────────────────────►│   ESP32 (Rover)     │
│  - Motor Control    │                              │  - Servo Gimbal     │
│  - Ultrasonic       │                              │  - MPU6050 IMU      │
│  - L298N Driver     │                              │  - Remote Control   │
└─────────────────────┘                              └─────────────────────┘

┌─────────────────────┐
│  ESP32 (Sensor Box) │
│  - DHT22 Temp/Hum   │
│  - NEO-6M GPS       │
│  - MQ-2 / MQ-7      │
│  - MQ-135           │
└─────────────────────┘
```

---

## 🧩 Components Overview

| # | Microcontroller | Role |
|---|----------------|------|
| 1 | Arduino Uno R3 | Motor driver, ultrasonic obstacle detection |
| 2 | ESP32 (Rover) | Servo gimbal control, MPU6050 stabilization, remote control |
| 3 | ESP32 (Sensor Box) | DHT22, GPS, MQ gas sensors |

---

## 🔌 Wiring Diagrams

### 1️⃣ ESP32 — Sensor Box

#### DHT22 (Temperature & Humidity)

| DHT22 Pin | ESP32 Pin | Notes |
|-----------|-----------|-------|
| VCC | 3.3V | — |
| DATA | GPIO15 | 10kΩ pull-up resistor between DATA and 3.3V |
| GND | GND | — |

#### NEO-6M GPS Module

| NEO-6M Pin | ESP32 Pin | Notes |
|------------|-----------|-------|
| VCC | 3.3V | — |
| GND | GND | — |
| TX | GPIO16 | ESP32 UART2 RX |
| RX | GPIO17 | ESP32 UART2 TX |

#### MQ Gas Sensors (MQ-2, MQ-7, MQ-135)

> ⚠️ **Important:** These sensors require **5V** for the heater coil. A **voltage divider** is mandatory on the analog output (5V → 2.5V) to protect the ESP32 ADC pins.

**Voltage Divider circuit:** `AOUT → [10kΩ] → ADC Pin → [10kΩ] → GND`

| Sensor | VCC | GND | AOUT (via divider) | ESP32 ADC Pin |
|--------|-----|-----|--------------------|---------------|
| MQ-2  | 5V | GND | Voltage divider | GPIO4 |
| MQ-7  | 5V | GND | Voltage divider | GPIO36 (input only) |
| MQ-135 | 5V | GND | Voltage divider | GPIO39 (input only) |

---

### 2️⃣ ESP32 — Rover Side

#### MPU6050 IMU (Upper Servo Stabilization)

| MPU6050 Pin | ESP32 Pin | Notes |
|-------------|-----------|-------|
| VCC | 3.3V | — |
| GND | GND | — |
| SDA | GPIO21 | I2C Data |
| SCL | GPIO22 | I2C Clock |
| AD0 | GND | Sets I2C address to 0x68 |

#### Servo Motors

| Servo | ESP32 GPIO | Description |
|-------|------------|-------------|
| Base / Lower Servo | GPIO18 | Horizontal rotation |
| Upper Servo | GPIO19 | Vertical tilt (stabilized by MPU6050) |

#### UART Link to Arduino Uno R3

| ESP32 Pin | Arduino Uno Pin | Notes |
|-----------|-----------------|-------|
| RX2 (GPIO16) | TX | Serial communication |
| TX2 (GPIO17) | RX | Serial communication |

> ⚠️ Use a **logic level shifter** if needed — the ESP32 is 3.3V logic and the Uno R3 is 5V logic.

---

### 3️⃣ Arduino Uno R3

#### L298N Motor Driver — 4× TT Motors

| L298N Pin | Uno Pin / Power |
|-----------|----------------|
| IN1 | Digital pin (Motor A direction) |
| IN2 | Digital pin (Motor A direction) |
| IN3 | Digital pin (Motor B direction) |
| IN4 | Digital pin (Motor B direction) |
| ENA | PWM pin (Motor A speed) |
| ENB | PWM pin (Motor B speed) |
| VCC | 7–12V battery |
| GND | Common GND |
| 5V out | Arduino 5V (optional) |

#### HC-SR04 Ultrasonic Sensors (×3)

| Sensor | TRIG Pin | ECHO Pin |
|--------|----------|----------|
| Front | Digital pin | Digital pin |
| Left  | Digital pin | Digital pin |
| Right | Digital pin | Digital pin |

> Exact Uno pin numbers are defined in the Uno firmware source file.

#### UART Link to ESP32 (Rover)

| Uno Pin | ESP32 (Rover) Pin |
|---------|-------------------|
| TX (pin 1) | RX2 |
| RX (pin 0) | TX2 |

---

## 📂 Repository Structure

```
surveillance-rover/
│
├── sensor_box/               # ESP32 Sensor Box firmware
│   └── sensor_box.ino        # DHT22, GPS, MQ sensors
│
├── rover_esp32/              # ESP32 Rover firmware
│   └── rover_esp32.ino       # Servo gimbal, MPU6050, remote control
│
├── rover_uno/                # Arduino Uno R3 firmware
│   └── rover_uno.ino         # Motors, ultrasonic sensors, L298N
│
├── wiring/                   # Wiring reference diagrams
│   ├── sensor_box_wiring.md
│   ├── rover_esp32_wiring.md
│   └── rover_uno_wiring.md
│
└── README.md
```

---

## 🛠️ Libraries & Dependencies

### ESP32 — Sensor Box

| Library | Purpose |
|---------|---------|
| `TinyGPSPlus` | Parse NMEA sentences from NEO-6M |
| `DHT sensor library` (Adafruit) | Read DHT22 temperature & humidity |
| `HardwareSerial` | UART2 for GPS communication |

### ESP32 — Rover

| Library | Purpose |
|---------|---------|
| `ESP32Servo` | PWM servo control |
| `Adafruit MPU6050` | IMU data for gimbal stabilization |
| `Wire` | I2C communication |

### Arduino Uno R3

| Library | Purpose |
|---------|---------|
| `NewPing` or custom | HC-SR04 ultrasonic sensors |
| Built-in | L298N motor control via `digitalWrite` / `analogWrite` |

---

## ⚙️ Setup & Upload

1. **Install Arduino IDE** (v2.x recommended) or **PlatformIO**
2. **Add ESP32 board support:**
   - Board Manager URL: `https://dl.espressif.com/dl/package_esp32_index.json`
3. **Install required libraries** via Library Manager (see above)
4. **Flash each controller separately:**
   - Flash `sensor_box.ino` → ESP32 Sensor Box
   - Flash `rover_esp32.ino` → ESP32 Rover
   - Flash `rover_uno.ino` → Arduino Uno R3
5. **Power up** and open Serial Monitor at `115200` baud to debug

---

## 📡 GPS Notes

- The NEO-6M may take **30–90 seconds** for a first satellite fix (cold start)
- Run outdoors or near a window for best signal
- Serial Monitor will show `⚠ WARNING: No data from GPS` if TX/RX wiring is incorrect
- HDOP < 2.0 indicates a good quality fix

---

## ⚠️ Important Hardware Warnings

- **MQ sensors need 5V** — never connect their VCC to 3.3V
- **Voltage divider is mandatory** on all MQ analog outputs before connecting to ESP32 ADC
- **GPIO36 and GPIO39** on ESP32 are input-only — do not drive them as outputs
- **Logic level shifter** may be required on the ESP32 ↔ Uno UART link (3.3V vs 5V)
- **AD0 → GND** on MPU6050 sets I2C address to `0x68` — tie high for `0x69` if needed
- Ensure **common GND** across all three controllers

---

## 🚀 Future Improvements

- [ ] Wi-Fi / MQTT telemetry from sensor box to dashboard
- [ ] Live video streaming from gimbal-mounted camera
- [ ] PID-based gimbal stabilization using MPU6050
- [ ] Web-based remote control interface
- [ ] SD card logging for GPS tracks and sensor data

---

## 📄 License

MIT License — feel free to use and modify for personal and educational projects.