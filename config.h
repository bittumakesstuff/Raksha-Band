
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>


#define SIMULATION_MODE 0

// Pin definitions
#define PIN_STATUS_LED    21 // Status LED indicator
#define PIN_BUZZER        3  // Alarm buzzer pin
#define PIN_TAMPER        2  // Strap loop pin (INPUT_PULLUP: LOW=intact, HIGH=cut)
#define PIN_BUTTON        15 // Manual SOS button (INPUT_PULLUP: LOW=pressed)
#define PIN_BT_BUTTON     16 // Bluetooth toggle button (INPUT_PULLUP: LOW=pressed for 2s)
#define PIN_RESET_BUTTON  17 // Reset connection button (INPUT_PULLUP: LOW=pressed for 2s)
#define PIN_BATTERY_ADC   1  // Battery voltage ADC pin

// Serial peripherals
#define PIN_GSM_RX        5  // SIM800L TX -> ESP32 RX1
#define PIN_GSM_TX        4  // SIM800L RX -> ESP32 TX1
#define PIN_GPS_RX        7  // NEO-6M TX -> ESP32 RX2
#define PIN_GPS_TX        6  // NEO-6M RX -> ESP32 TX2

// Audio (I2S Microphone INMP441)
#define PIN_I2S_WS        42 // Word Select / Frame Sync
#define PIN_I2S_SD        41 // Serial Data Out
#define PIN_I2S_SCK       43 // Serial Clock

// I2C bus (MPU6050 & MAX30102)
#define PIN_I2C_SDA       8
#define PIN_I2C_SCL       9

// Default contact numbers
#define DEFAULT_EMERGENCY_PHONE "+919876543210"
#define POLICE_PHONE_NUMBER     "112"

// Timing configs (ms)
#define CANCEL_WINDOW_MS        20000UL // 20s cancel window
#define GPS_UPDATE_INTERVAL_MS  45000UL // 45s GPS update during alarm
#define BT_ENABLE_HOLD_MS       2000UL  // Hold GPIO 16 for 2s to start BT pairing
#define BT_RESET_HOLD_MS        2000UL  // Hold GPIO 17 for 2s to reset phone connection

// Thresholds
#define FALL_G_THRESHOLD        3.2f
#define SHAKE_G_THRESHOLD       2.5f
#define SUSTAINED_SHAKE_MS      600
#define GYRO_STRUGGLE_THRESHOLD 250.0f  // Deg/sec for violent rotation/struggle
#define AUDIO_SAMPLE_RATE       16000
#define SCREAM_AMPLITUDE_THRES  18000
#define HR_SPIKE_THRESHOLD_BPM  130

// BLE UUIDs for App Connection Interface
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_BATTERY_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CHAR_PHONE_UUID     "1c95d5e3-04b7-4173-9f08-60b24016a20d"
#define BLE_CHAR_ALERT_UUID     "d292a031-6b80-494d-8d45-7171082c3c47"
#define BLE_CHAR_SERIAL_UUID    "e376a1c4-1122-48cb-9966-5197825e364e"

// Default test coordinates
#define DEFAULT_SIM_LAT         28.613939
#define DEFAULT_SIM_LNG         77.209021

#endif
