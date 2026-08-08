/*
 * Raksha Band - Safety Bracelet Configuration
 * Target MCU: ESP32-C3 (RISC-V)
 * Peripherals: SIM7000E (LTE-M/NB-IoT + GNSS), LIS3DH (I2C), Reed switch, 
 *              Vibration motor, Status LED/Buzzer, TP4056 + LiPo, BT Switch.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Debug serial baud rate
#define DEBUG_BAUD            115200

// Feature Toggles
#define ENABLE_BLE            1  // Bluetooth App & Switch Control Enabled

// I2C Pins for LIS3DH Accelerometer
#define PIN_SDA               8
#define PIN_SCL               9
#define LIS3DH_ADDR_PRIMARY   0x18
#define LIS3DH_ADDR_ALT       0x19

// SIM7000E UART & Power Control
#define PIN_SIM_TX            4  // ESP32-C3 TX (GPIO4) -> SIM7000E RXD
#define PIN_SIM_RX            5  // ESP32-C3 RX (GPIO5) -> SIM7000E TXD
#define PIN_SIM_PWRKEY        3  // Pulse LOW to boot/wake SIM7000E
#define SIM_UART_BAUD         115200

// Hardware Triggers & Outputs
#define PIN_REED_SW           2  // Reed switch magnet / tamper sensor
#define PIN_LIS3DH_INT        7  // Motion / Fall hardware interrupt
#define PIN_VIBRO_MOTOR       6  // Transistor base pin for vibration motor
#define PIN_STATUS_LED        10 // Status LED / Buzzer alert output
#define PIN_BATTERY_ADC       1  // Battery voltage sense pin
#define PIN_BT_SWITCH         0  // Dedicated Bluetooth ON/OFF Switch (Active LOW: BT ON, HIGH: BT OFF)

// System Timing Parameters (ms)
#define CANCEL_WINDOW_MS      20000UL // 20-second emergency cancel window
#define BT_DEBOUNCE_MS        300UL   // Debounce window for BT ON/OFF switch
#define BT_HOLD_RESET_MS      3000UL  // 3s hold to reset phone pairing lock
#define DEEP_SLEEP_TIMEOUT_MS 30000UL // Inactivity delay before sleeping

// Motion & Fall Detection Thresholds
#define IMPACT_G_THRESHOLD    3.2f    // Impact spike G-force trigger
#define STILLNESS_TIME_MS     1200    // Post-impact stillness window
#define STRUGGLE_VAR_THRES    2.8f    // High-variance erratic struggle threshold
#define STRUGGLE_SUSTAIN_MS   800     // Sustained struggle duration

// Reed Switch Operating Modes
// 1 = Magnet swipe trigger (Active LOW: magnet near switch triggers alert)
// 0 = Clasp tamper / removal trigger (Active HIGH: clasp opened triggers alert)
#define REED_MODE_SWIPE       1

// Default Contacts & Emergency Info
#define DEFAULT_PHONE_NUMBER  "+919876543210"
#define NVS_NAMESPACE         "raksha"

// BLE UUID Definitions for Mobile App Integration
#define BLE_SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_BATTERY_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CHAR_PHONE_UUID   "1c95d5e3-04b7-4173-9f08-60b24016a20d"
#define BLE_CHAR_ALERT_UUID   "d292a031-6b80-494d-8d45-7171082c3c47"
#define BLE_CHAR_SERIAL_UUID  "e376a1c4-1122-48cb-9966-5197825e364e"

#endif // CONFIG_H
