/*
 * ====================================================================================
 * RAKSHA BAND - ESP32-S3 SAFETY BRACELET FIRMWARE
 * Configuration & Pin Mapping Header File
 * ====================================================================================
 * Based on Raksha Band Connection Sheet & Hardware Specs
 * ESP32-S3-WROOM-1 / DevKit Pinout
 * ====================================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 0. SIMULATION / WOKWI TEST MODE TOGGLE
// ==========================================
// Set to 1 to enable Wokwi / Simulator compatibility mode & Serial test commands
// Set to 0 for production real hardware deployment
#define SIMULATION_MODE         1   

// ==========================================
// 1. PIN DEFINITIONS (MATCHING CONNECTION SHEET)
// ==========================================

// Status LED (Indicator / Siren Visual)
#define PIN_STATUS_LED          21  // Anode (+) via 220-330 ohm resistor

// Buzzer (Audible Siren)
#define PIN_BUZZER              3   // Positive (+), Negative to GND

// Tamper Switch (Strap Loop Integrity)
#define PIN_TAMPER              2   // Uses INPUT_PULLUP. LOW = Strap Intact, HIGH = Strap Tampered/Cut

// Manual SOS / Setup Button
#define PIN_BUTTON              15  // Uses INPUT_PULLUP. LOW = Pressed, HIGH = Released

// Battery Monitoring (ADC)
#define PIN_BATTERY_ADC         1   // Middle wiper of potentiometer / battery divider (ADC1_CH0)

// SIM800L GSM Module (HardwareSerial 1)
#define PIN_GSM_RX              5   // Connected to SIM800L TX
#define PIN_GSM_TX              4   // Connected to SIM800L RX

// NEO-6M GPS Module (HardwareSerial 2)
#define PIN_GPS_RX              7   // Connected to GPS TX
#define PIN_GPS_TX              6   // Connected to GPS RX

// INMP441 I2S Microphone
#define PIN_I2S_WS              42  // Word Select / Frame Sync (L/R Clock)
#define PIN_I2S_SD              41  // Serial Data Out
#define PIN_I2S_SCK             43  // Serial Clock (BCLK)

// Shared I2C Bus (MPU6050 & MAX30102 / MAX30105)
#define PIN_I2C_SDA             8   // Default SDA pin for ESP32-S3
#define PIN_I2C_SCL             9   // Default SCL pin for ESP32-S3

// ==========================================
// 2. EMERGENCY CONTACT & SYSTEM CONFIGURATION
// ==========================================

// Primary Emergency Contact Phone Number (Include Country Code, e.g., "+919876543210")
#define EMERGENCY_PHONE_NUMBER "+919876543210"

// Police Helpline Number (for auto-escalation voice call)
#define POLICE_PHONE_NUMBER    "112"

// Thresholds & Timers
#define CANCEL_WINDOW_MS       20000UL  // 20 Seconds to receive CANCEL SMS or press button
#define GPS_UPDATE_INTERVAL_MS 45000UL  // 45 Seconds periodic location update during alarm

// Motion & Sensor Fusion Thresholds
#define FALL_G_THRESHOLD       3.2f     // G-force threshold for fall/impact detection (> 3.2G)
#define SHAKE_G_THRESHOLD      2.5f     // G-force threshold for violent shake (> 2.5G sustained)
#define SUSTAINED_SHAKE_MS     600      // Required duration for violent shake (0.4s - 1.2s range)

// Audio Thresholds (I2S Mic)
#define AUDIO_SAMPLE_RATE      16000    // 16 kHz sampling rate for voice / scream
#define SCREAM_AMPLITUDE_THRES 18000    // Amplitude threshold for scream detection

// Heart Rate Thresholds (MAX30102)
#define HR_SPIKE_THRESHOLD_BPM 130      // Heart rate spike threshold (BPM)

// Simulated GPS Coordinates (for Wokwi / indoor testing without satellite lock)
#define DEFAULT_SIM_LAT        28.613939
#define DEFAULT_SIM_LNG        77.209021

#endif // CONFIG_H
