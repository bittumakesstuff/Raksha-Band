# 🛡️ Raksha Band - ESP32-S3 Acoustic & Multi-Sensor Safety Bracelet

This repository contains the C++ firmware for the **Raksha Band** wearable safety bracelet, built for the **ESP32-S3 Microcontroller** (ESP32-S3-WROOM-1 DevKit).

---

## 🖼️ Circuit & Wiring Diagram

![Raksha Band Circuit Diagram](circuit.png)

---

## 📌 Pinout & Connection Mapping Table

| Component | Component Pin | ESP32-S3 Pin | Notes & Wiring Requirements |
| :--- | :--- | :--- | :--- |
| **Status LED** | Anode (+) | **GPIO 21** | Series resistor ($220\Omega - 330\Omega$). Cathode to GND. |
| **Siren Buzzer** | Positive (+) | **GPIO 3** | Negative (-) to GND. |
| **Tamper Switch** | Terminal 1 | **GPIO 2** | `INPUT_PULLUP`. LOW = Intact, HIGH = Strap cut/tampered. |
| **Manual SOS / Setup Button** | Terminal 1 | **GPIO 15** | `INPUT_PULLUP`. Short press = Setup, Long press (>2s) = Manual SOS. |
| **Bluetooth Toggle Button** | Terminal 1 | **GPIO 16** | `INPUT_PULLUP`. Hold 2 seconds to enable Bluetooth advertising & pairing. |
| **Reset Connection Button** | Terminal 1 | **GPIO 17** | `INPUT_PULLUP`. Hold 2 seconds to clear paired mobile phone serial from NVS memory. |
| **Battery Voltage (ADC)** | Wiper (Middle) | **GPIO 1** | ADC1_CH0 voltage divider. |
| **SIM800L (GSM)** | TX / RX | **GPIO 5 / GPIO 4** | HardwareSerial 1 (RX: 5, TX: 4). Sends SOS SMS & auto calls. |
| **NEO-6M (GPS)** | TX / RX | **GPIO 7 / GPIO 6** | HardwareSerial 2 (RX: 7, TX: 6). Provides latitude & longitude. |
| **INMP441 (I2S Mic)** | WS / SD / SCK | **GPIO 42 / 41 / 43** | I2S Microphone for dynamic noise cancellation & scream SOS. |
| **MPU6050 (IMU)** | SDA / SCL | **GPIO 8 / GPIO 9** | Shared I2C bus for fall & kidnapping/struggle motion detection. |
| **MAX30102** | SDA / SCL | **GPIO 8 / GPIO 9** | Shared I2C bus for biometric heart rate spike monitoring. |

---

## 📱 Bluetooth & Mobile App Integration

1. **Bluetooth Enable (`GPIO 16`)**:
   - User presses the Bluetooth button for 2 seconds. ESP32-S3 starts BLE advertising under device serial `RB-S3-XXXX`.
2. **Unique Serial & Mobile Pairing Lock**:
   - The device assigns itself a unique serial derived from its MAC address.
   - Upon connection, the mobile app transmits its unique serial. The Raksha Band locks pairing to that phone serial in Non-Volatile Storage (NVS).
3. **Reset Connection (`GPIO 17`)**:
   - To connect the Raksha Band to a new mobile device (e.g. re-assigning to another family member), press the Reset button (`GPIO 17`) for 2 seconds to clear the stored paired phone serial.
4. **Standalone Operation**:
   - If out of Bluetooth range, the Bluetooth connection disconnects, but Raksha Band continues operating standalone (monitoring scream, motion, tamper loop, and sending GSM alerts).
5. **App Communication Ports (BLE GATT Service)**:
   - **Service UUID**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
   - **Battery Level Characteristic** (`beb5483e-36e1-4688-b7f5-ea07361b26a8`): Notifies battery percentage.
   - **Emergency Contact Characteristic** (`1c95d5e3-04b7-4173-9f08-60b24016a20d`): Allows the mobile app to configure/update the emergency contact phone number dynamically.
   - **Alert History Characteristic** (`d292a031-6b80-494d-8d45-7171082c3c47`): Transmits alert history logs with GPS latitude, longitude, and trigger source.
   - **Device Serial Lock Characteristic** (`e376a1c4-1122-48cb-9966-5197825e364e`): Handles handshake and pairing lock.

---

## 🔊 Dynamic Noise Cancellation & Scream SOS

- Tracks dynamic background ambient noise floor via an Exponential Moving Average (EMA).
- Discriminates human screams from ambient noise spikes by checking audio peaks against the dynamic noise floor.
- Fires emergency GSM SMS with live GPS latitude and longitude upon scream detection.

---

## 🏃 Motion Classification (Walking/Running vs. Kidnapping/Struggle)

- **Walking / Running**: Rhythmic, periodic acceleration spikes (1.2G - 2.2G) with low rotational angular velocity are recognized as gait steps and filtered out to prevent false alarms.
- **Kidnapping / Violent Struggle**: High G-force spikes (>2.5G) combined with high gyroscope angular velocity (>250 deg/sec) trigger an emergency alarm.

---
