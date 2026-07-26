# 🛡️ Raksha Band - ESP32-S3 Acoustic AI & Multi-Sensor Safety Bracelet

This repository contains the complete, production-ready C++ firmware for the **Raksha Band** wearable safety bracelet, developed for the **ESP32-S3 Microcontroller** (ESP32-S3-WROOM-1 DevKit).

---

## 🖼️ Circuit & Wiring Diagram

![Raksha Band Circuit Diagram](Screenshot%202026-07-26%20142841.png)

---

## 📌 Pinout & Connection Mapping Table

All connections strictly follow the **Raksha Band Connection Sheet** and hardware schematic:

| Component | Component Pin | ESP32-S3 Pin | Notes & Wiring Requirements |
| :--- | :--- | :--- | :--- |
| **Status LED** | Anode (+) | **GPIO 21** | Add a $220\Omega - 330\Omega$ resistor in series. Connect Cathode (-) to GND. |
| **Siren Buzzer** | Positive (+) | **GPIO 3** | Connect Negative (-) to GND. |
| **Tamper Switch** | Terminal 1 | **GPIO 2** | Connect Terminal 2 to GND (`INPUT_PULLUP`). LOW = Intact, HIGH = Cut/Tampered. |
| **Manual SOS / Setup Button** | Terminal 1 | **GPIO 15** | Connect Terminal 2 to GND (`INPUT_PULLUP`). LOW = Pressed. |
| **Battery Voltage (ADC)** | Wiper (Middle) | **GPIO 1** | ADC1_CH0. Outer legs to 3.3V and GND. |
| **SIM800L (GSM)** | TX | **GPIO 5** | ESP32 HardwareSerial 1 RX (Receives data from SIM800L). |
| | RX | **GPIO 4** | ESP32 HardwareSerial 1 TX (Transmits data to SIM800L). |
| **NEO-6M (GPS)** | TX | **GPIO 7** | ESP32 HardwareSerial 2 RX (Receives data from NEO-6M). |
| | RX | **GPIO 6** | ESP32 HardwareSerial 2 TX (Transmits data to NEO-6M). |
| **INMP441 (I2S Mic)** | WS (L/R Clock) | **GPIO 42** | I2S Word Select / Frame Sync. |
| | SD (Data Out) | **GPIO 41** | I2S Serial Data Out. |
| | SCK (BCLK) | **GPIO 43** | I2S Serial Clock. |
| | L/R Channel | **GND** | Left Channel format (`I2S_CHANNEL_FMT_ONLY_LEFT`). |
| **MPU6050 (IMU)** | SDA | **GPIO 8** | Default I2C SDA pin for ESP32-S3. |
| | SCL | **GPIO 9** | Default I2C SCL pin for ESP32-S3. |
| **MAX30102 / MAX30105** | SDA | **GPIO 8** | Shared I2C SDA bus. |
| | SCL | **GPIO 9** | Shared I2C SCL bus. |

---

## ⚙️ Trigger Fusion Logic & System Modes

### **Modes of Operation**
1. **`NORMAL_MODE`**: Continuous non-blocking monitoring of all acoustic, biometric, motion, tamper loop, battery, and button inputs.
2. **`SETUP_MODE`**: Triggered by a short press on the Manual SOS Button (GPIO 15). Blinks LED rapidly for baseline calibration.
3. **`ALARM_MODE`**: Emergency triggered! Activates local siren (Buzzer & LED), fetches live GPS, fires instant SOS SMS with Google Maps link, runs a **20-second escalation window** for disarm/cancel, auto-calls emergency contact if no cancel is received, and sends periodic location updates every 45s.

### **Sensor Fusion Rules**
- 🔴 **Tamper Loop Broken** (`GPIO 2` goes `HIGH`): Fires **instantly** (no confirmation needed).
- 🔴 **Safe-word Pattern Matched** (Acoustic energy / DTW): Fires **instantly** (deliberate high-confidence trigger).
- 🔴 **Manual SOS Long Press** (`GPIO 15` held > 2s): Fires **instantly**.
- 🟠 **Scream Classified** (I2S mic peak > threshold): Requires corroboration (Motion G-spike OR Heart Rate spike within 3s).
- 🟠 **Violent Shake / Struggle / Fall** ($G > 2.5G$ sustained over 0.4s – 1.2s): Fires after confirmation window.
- 🟡 **Heart Rate Spike Alone**: Sensor fusion corroboration signal only (never triggers alone).

---

## 📦 Required Libraries

In the Arduino IDE Library Manager, install:
1. **`Adafruit MPU6050`** by Adafruit
2. **`MAX30105 Pulse and Proximity Sensor Library`** by SparkFun
3. **`TinyGPSPlus`** by Mikal Hart

---

## 🚀 How to Build & Upload

1. Open `Raksha_Band_ESP32S3.ino` in Arduino IDE or VS Code with PlatformIO.
2. Open `config.h` and update `EMERGENCY_PHONE_NUMBER` with your emergency target number (e.g. `+919876543210`).
3. Select Board: **ESP32S3 Dev Module** (Partition Scheme: Default 4MB with SPIFFS / Huge APP).
4. Select Port and click **Upload**.
5. Open Serial Monitor at **115200 baud**.
