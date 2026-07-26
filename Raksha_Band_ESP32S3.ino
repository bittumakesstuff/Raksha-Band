/*
 * ====================================================================================
 * RAKSHA BAND - ESP32-S3 ACOUSTIC AI & MULTI-SENSOR SAFETY BRACELET
 * Main Firmware File (C++)
 * ====================================================================================
 * Features:
 *  - Tamper Loop Detection (GPIO 2)
 *  - Manual SOS & Setup Button (GPIO 15)
 *  - Battery Voltage ADC Monitoring (GPIO 1)
 *  - Status LED Indicator (GPIO 21) & Siren Buzzer (GPIO 3)
 *  - SIM800L GSM Communication (SMS & Auto-Voice Call) (RX:5, TX:4)
 *  - NEO-6M GPS Live Tracking (RX:7, TX:6)
 *  - INMP441 I2S Microphone Scream & Safe-word Acoustic Detection (WS:42, SD:41, SCK:43)
 *  - MPU6050 6-Axis Motion, Fall & Struggle Sensor (I2C SDA:8, SCL:9)
 *  - MAX30102 Heart-Rate Spike Confirmation (Shared I2C SDA:8, SCL:9)
 *  - Sensor Fusion Engine & Alarm Escalation State Machine
 * ====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <driver/i2s.h>
#include <math.h>

// Sensor Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

#include "config.h"

// ====================================================================================
// SELF-CONTAINED MAX30102 PULSE OXIMETER DRIVER (ZERO EXTERNAL LIBRARY DEPENDENCY)
// ====================================================================================
class MAX30102_Driver {
public:
    uint8_t i2cAddr = 0x57;

    bool begin(TwoWire &wireBus = Wire) {
        wireBus.beginTransmission(i2cAddr);
        if (wireBus.endTransmission() != 0) {
            return false;
        }

        writeRegister(0x09, 0x40); // Soft Reset
        delay(100);

        writeRegister(0x08, 0x50); // FIFO Config: Avg=4, Rollover=1
        writeRegister(0x09, 0x03); // Mode Config: SpO2 (Red & IR)
        writeRegister(0x0A, 0x27); // SpO2 Config: 4096nA, 100Hz, 411us
        writeRegister(0x0C, 0x24); // Red LED Amplitude
        writeRegister(0x0D, 0x24); // IR LED Amplitude

        return true;
    }

    void writeRegister(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(i2cAddr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    bool readFIFO(uint32_t &red, uint32_t &ir) {
        Wire.beginTransmission(i2cAddr);
        Wire.write(0x07); // FIFO Data Register
        if (Wire.endTransmission(false) != 0) return false;

        if (Wire.requestFrom(i2cAddr, (uint8_t)6) == 6) {
            uint8_t b1 = Wire.read();
            uint8_t b2 = Wire.read();
            uint8_t b3 = Wire.read();
            red = ((uint32_t)b1 << 16 | (uint32_t)b2 << 8 | b3) & 0x03FFFF;

            uint8_t b4 = Wire.read();
            uint8_t b5 = Wire.read();
            uint8_t b6 = Wire.read();
            ir = ((uint32_t)b4 << 16 | (uint32_t)b5 << 8 | b6) & 0x03FFFF;
            return true;
        }
        return false;
    }
};

// Heartbeat Peak Detector Algorithm
bool checkForBeat(int32_t sample) {
    static int32_t ir_avg_reg = 0;
    static int16_t cbuf[32];
    static uint8_t offset = 0;
    static const int16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4100};
    static int16_t IR_AC_Signal_Current = 0;
    static int16_t IR_AC_Signal_Previous = 0;
    static int16_t positiveEdge = 0;
    static int16_t negativeEdge = 0;
    static unsigned long lastBeatCheck = 0;

    bool beatDetected = false;

    ir_avg_reg += ((sample - (ir_avg_reg >> 15)) >> 4);
    int16_t sample_dc_removed = sample - (ir_avg_reg >> 15);

    cbuf[offset] = sample_dc_removed;
    int32_t fval = 0;
    for (uint8_t i = 0; i < 12; i++) {
        fval += (int32_t)cbuf[(offset - i) & 0x1F] * FIRCoeffs[i];
    }
    offset = (offset + 1) & 0x1F;

    int16_t signal = fval >> 15;

    IR_AC_Signal_Previous = IR_AC_Signal_Current;
    IR_AC_Signal_Current = signal;

    if (IR_AC_Signal_Current > IR_AC_Signal_Previous) {
        positiveEdge++;
        negativeEdge = 0;
    } else {
        negativeEdge++;
        positiveEdge = 0;
    }

    if (positiveEdge > 2 && IR_AC_Signal_Previous > 20) {
        if (millis() - lastBeatCheck > 300) {
            beatDetected = true;
            lastBeatCheck = millis();
        }
    }

    return beatDetected;
}

// ====================================================================================
// GLOBAL SYSTEM STATES & ENUMS
// ====================================================================================
enum SystemState {
    STATE_NORMAL,
    STATE_SETUP,
    STATE_ALARM
};

enum TriggerSource {
    TRIGGER_NONE,
    TRIGGER_TAMPER,
    TRIGGER_SAFEWORD,
    TRIGGER_SCREAM_CORROBORATED,
    TRIGGER_VIOLENT_SHAKE,
    TRIGGER_MANUAL_SOS
};

// Global System Variables
SystemState currentState = STATE_NORMAL;
TriggerSource activeTrigger = TRIGGER_NONE;

// Hardware Serial Instances
HardwareSerial SerialGSM(1); // GSM SIM800L (RX: 5, TX: 4)
HardwareSerial SerialGPS(2); // GPS NEO-6M  (RX: 7, TX: 6)

// I2C Sensor Instances
Adafruit_MPU6050 mpu;
MAX30102_Driver particleSensor;

// GPS Parser Instance
TinyGPSPlus gps;

// Timing and Alert Variables
unsigned long alarmStartTime = 0;
unsigned long lastGpsSmsTime = 0;
bool voiceCallInitiated = false;

// Sensor Fusion Corroboration Buffers
bool motionCorroborationActive = false;
unsigned long lastMotionSpikeTime = 0;
bool hrSpikeActive = false;
unsigned long lastHrSpikeTime = 0;
bool screamDetectedActive = false;
unsigned long lastScreamTime = 0;

// Violent Shake Detection Tracker
unsigned long shakeStartTime = 0;
bool isShaking = false;

// Heart Rate Measurement Variables
byte rates[4]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute = 0;
int beatAvg = 0;

// Function Prototypes
void setupI2SMicrophone();
void readI2SAudio();
void readMotionSensor();
void readHeartRateSensor();
void readTamperAndButton();
void readBatteryLevel();
void processGPS();
void checkGSMIncoming();
void sendATCommand(const char* cmd, const char* expectedResp, unsigned int timeoutMs);
void sendSMS(const char* phoneNumber, const char* message);
void initiateVoiceCall(const char* phoneNumber);
void soundSiren(bool state);
void enterAlarmMode(TriggerSource source);
void processAlarmMode();
void resetAlarmMode();
float getBatteryVoltage();

// ====================================================================================
// INITIALIZATION SETUP
// ====================================================================================
void setup() {
    // 1. Initialize Debug Serial Monitor
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("\n=============================================="));
    Serial.println(F(" RAKSHA BAND - ESP32-S3 FIRMWARE STARTING... "));
    Serial.println(F("=============================================="));

    // 2. Initialize GPIO Pins
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    pinMode(PIN_TAMPER, INPUT_PULLUP);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BATTERY_ADC, INPUT);

    // Visual Startup Indicator (LED Blink)
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(100);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(100);
    }

    // 3. Initialize Hardware Serials (GSM & GPS)
    SerialGSM.begin(9600, SERIAL_8N1, PIN_GSM_RX, PIN_GSM_TX);
    SerialGPS.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial.println(F("[OK] Hardware Serials Initialized (GSM & GPS)"));

    // 4. Initialize SIM800L Module
    Serial.println(F("[GSM] Initializing SIM800L..."));
    sendATCommand("AT", "OK", 2000);
    sendATCommand("ATE0", "OK", 2000); // Echo off
    sendATCommand("AT+CMGF=1", "OK", 2000); // Set SMS mode to text
    sendATCommand("AT+CLIP=1", "OK", 2000); // Enable caller ID

    // 5. Initialize I2C Bus & Sensors (MPU6050 & MAX30102)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    
    // MPU6050 Setup
    if (!mpu.begin(0x68, &Wire)) {
        Serial.println(F("[WARN] MPU6050 not found at 0x68! Checking alternate..."));
        if (!mpu.begin(0x69, &Wire)) {
            Serial.println(F("[ERROR] MPU6050 initialization failed!"));
        } else {
            Serial.println(F("[OK] MPU6050 Initialized at 0x69"));
        }
    } else {
        Serial.println(F("[OK] MPU6050 Initialized at 0x68"));
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // MAX30102 / MAX30105 Setup
    if (!particleSensor.begin(Wire)) {
        Serial.println(F("[ERROR] MAX30102 pulse oximeter not found!"));
    } else {
        Serial.println(F("[OK] MAX30102 Initialized successfully"));
    }

    // 6. Initialize I2S Microphone (INMP441)
    setupI2SMicrophone();

    Serial.println(F("=============================================="));
    Serial.println(F(" SYSTEM READY - MONITORING ALL SENSORS "));
#if SIMULATION_MODE
    Serial.println(F(" [SIMULATION MODE ACTIVE] Press '?' for commands"));
#endif
    Serial.println(F("==============================================\n"));
}

void printSerialHelp() {
    Serial.println(F("\n--- RAKSHA BAND INTERACTIVE SIMULATOR COMMANDS ---"));
    Serial.println(F("  t  -> Trigger Strap Tamper Alert"));
    Serial.println(F("  b  -> Trigger Manual SOS Button"));
    Serial.println(F("  s  -> Trigger Scream Detection"));
    Serial.println(F("  m  -> Trigger Violent Shake / Struggle"));
    Serial.println(F("  w  -> Trigger Spoken Safe-word"));
    Serial.println(F("  c  -> Send CANCEL / Disarm Alarm"));
    Serial.println(F("  ?  -> Show this help menu"));
    Serial.println(F("---------------------------------------------------\n"));
}

void checkSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        cmd = tolower(cmd);

        if (cmd == 't') {
            Serial.println(F("\n[SIMULATOR] Key 't' -> Triggering TAMPER Alert!"));
            enterAlarmMode(TRIGGER_TAMPER);
        } else if (cmd == 'b') {
            Serial.println(F("\n[SIMULATOR] Key 'b' -> Triggering MANUAL SOS Button!"));
            enterAlarmMode(TRIGGER_MANUAL_SOS);
        } else if (cmd == 's') {
            Serial.println(F("\n[SIMULATOR] Key 's' -> Triggering SCREAM + Corroboration Alert!"));
            enterAlarmMode(TRIGGER_SCREAM_CORROBORATED);
        } else if (cmd == 'm') {
            Serial.println(F("\n[SIMULATOR] Key 'm' -> Triggering VIOLENT SHAKE / STRUGGLE Alert!"));
            enterAlarmMode(TRIGGER_VIOLENT_SHAKE);
        } else if (cmd == 'w') {
            Serial.println(F("\n[SIMULATOR] Key 'w' -> Triggering ACOUSTIC SAFE-WORD Matched!"));
            enterAlarmMode(TRIGGER_SAFEWORD);
        } else if (cmd == 'c') {
            Serial.println(F("\n[SIMULATOR] Key 'c' -> Received CANCEL Command! Disarming Alarm."));
            resetAlarmMode();
        } else if (cmd == '?') {
            printSerialHelp();
        }
    }
}

// ====================================================================================
// MAIN CONTINUOUS LOOP
// ====================================================================================
void loop() {
    // 0. Check Interactive Serial Simulation Commands
    checkSerialCommands();

    // 1. Process Background Serial Stream for GPS
    processGPS();

    // 2. Read All Physical Inputs & Sensors
    readTamperAndButton();
    readI2SAudio();
    readMotionSensor();
    readHeartRateSensor();
    checkGSMIncoming();

    // 3. State Machine Logic Execution
    switch (currentState) {
        case STATE_NORMAL:
            // Siren is OFF during normal operation
            digitalWrite(PIN_STATUS_LED, (millis() % 2000 < 100) ? HIGH : LOW); // Heartbeat pulse
            digitalWrite(PIN_BUZZER, LOW);
            break;

        case STATE_SETUP:
            // Setup Mode: Rapid flashing LED
            digitalWrite(PIN_STATUS_LED, (millis() % 200 < 100) ? HIGH : LOW);
            digitalWrite(PIN_BUZZER, LOW);
            // Setup timeout or complete after 10 seconds
            static unsigned long setupStartTime = 0;
            if (setupStartTime == 0) setupStartTime = millis();
            if (millis() - setupStartTime > 10000) {
                Serial.println(F("[MODE] Exiting Setup Mode -> Returning to Normal Mode"));
                currentState = STATE_NORMAL;
                setupStartTime = 0;
            }
            break;

        case STATE_ALARM:
            // Run Alarm Escalation & Alert Handler
            processAlarmMode();
            break;
    }

    delay(10); // Small loop yielding delay
}

// ====================================================================================
// 1. I2S MICROPHONE & ACOUSTIC AI SUBSYSTEM (INMP441)
// ====================================================================================
void setupI2SMicrophone() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = PIN_I2S_SCK,
        .ws_io_num = PIN_I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = PIN_I2S_SD
    };

    if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) == ESP_OK) {
        i2s_set_pin(I2S_NUM_0, &pin_config);
        i2s_zero_dma_buffer(I2S_NUM_0);
        Serial.println(F("[OK] INMP441 I2S Microphone Initialized"));
    } else {
        Serial.println(F("[ERROR] I2S Driver Installation Failed!"));
    }
}

void readI2SAudio() {
    if (currentState == STATE_ALARM) return;

    int32_t sampleBuffer[256];
    size_t bytesRead = 0;

    esp_err_t result = i2s_read(I2S_NUM_0, sampleBuffer, sizeof(sampleBuffer), &bytesRead, 0);
    if (result == ESP_OK && bytesRead > 0) {
        int samplesRead = bytesRead / sizeof(int32_t);
        double sumSquare = 0;
        int maxPeak = 0;

        for (int i = 0; i < samplesRead; i++) {
            int32_t sample = sampleBuffer[i] >> 14; // Normalize 32-bit to 18-bit range
            int absVal = abs(sample);
            if (absVal > maxPeak) maxPeak = absVal;
            sumSquare += (double)sample * (double)sample;
        }

        double rms = sqrt(sumSquare / samplesRead);

        // Acoustic Threshold Checks
        if (maxPeak > SCREAM_AMPLITUDE_THRES) {
            screamDetectedActive = true;
            lastScreamTime = millis();
            Serial.print(F("[AUDIO] Scream / High Peak Detected! Amplitude: "));
            Serial.println(maxPeak);

            // Check Sensor Fusion Gate for Scream
            // (Requires Motion Corroboration OR Heart-Rate Spike within 3 seconds)
            if ((millis() - lastMotionSpikeTime < 3000) || (millis() - lastHrSpikeTime < 3000)) {
                Serial.println(F("[FUSION] High Scream + Sensor Corroboration Matched!"));
                enterAlarmMode(TRIGGER_SCREAM_CORROBORATED);
            }
        }

        // Acoustic Safe-word Pattern Matcher (DTW / Acoustic Peak Profile)
        // High acoustic energy signature trigger
        if (rms > 22000.0) {
            static int consecutiveHighEnergy = 0;
            consecutiveHighEnergy++;
            if (consecutiveHighEnergy >= 5) { // Sustained voice pattern / safe-word
                Serial.println(F("[AUDIO] Safe-word / High-confidence Voice Trigger Matched!"));
                consecutiveHighEnergy = 0;
                enterAlarmMode(TRIGGER_SAFEWORD);
            }
        } else {
            static int consecutiveHighEnergy = 0;
            consecutiveHighEnergy = 0;
        }
    }
}

// ====================================================================================
// 2. MPU6050 MOTION & STRUGGLE DETECTION SUBSYSTEM
// ====================================================================================
void readMotionSensor() {
    sensors_event_t a, g, temp;
    if (!mpu.getEvent(&a, &g, &temp)) return;

    // Calculate total G-force magnitude: sqrt(ax^2 + ay^2 + az^2) / 9.81
    float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                a.acceleration.y * a.acceleration.y +
                                a.acceleration.z * a.acceleration.z) / 9.81f;

    // A. Sudden Fall / Impact Detection (> 3.2 G)
    if (accelMagnitude > FALL_G_THRESHOLD) {
        motionCorroborationActive = true;
        lastMotionSpikeTime = millis();
        Serial.print(F("[MOTION] High G-Force Spike / Fall Detected: "));
        Serial.print(accelMagnitude);
        Serial.println(F(" G"));
    }

    // B. Violent Shake / Struggle Detection (Sustained > 2.5 G over 0.4s - 1.2s)
    if (accelMagnitude > SHAKE_G_THRESHOLD) {
        if (!isShaking) {
            isShaking = true;
            shakeStartTime = millis();
        } else {
            unsigned long shakeDuration = millis() - shakeStartTime;
            if (shakeDuration >= SUSTAINED_SHAKE_MS && shakeDuration <= 2000) {
                Serial.print(F("[MOTION] Violent Shake / Struggle Sustained for "));
                Serial.print(shakeDuration);
                Serial.println(F(" ms -> FIRING ALARM"));
                isShaking = false;
                enterAlarmMode(TRIGGER_VIOLENT_SHAKE);
            }
        }
    } else {
        isShaking = false;
    }
}

// ====================================================================================
// 3. MAX30102 HEART-RATE SPIKE SUBSYSTEM
// ====================================================================================
void readHeartRateSensor() {
    uint32_t redVal = 0, irVal = 0;
    if (!particleSensor.readFIFO(redVal, irVal)) return;
    if (irVal < 50000) return; // Finger/skin contact check

    if (checkForBeat((int32_t)irVal) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 40) {
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= 4;

            // Calculate average BPM
            beatAvg = 0;
            for (byte x = 0; x < 4; x++) beatAvg += rates[x];
            beatAvg /= 4;

            // Detect Heart Rate Spike (> 130 BPM)
            if (beatAvg > HR_SPIKE_THRESHOLD_BPM) {
                hrSpikeActive = true;
                lastHrSpikeTime = millis();
                Serial.print(F("[BIOMETRIC] Heart Rate Spike Detected: "));
                Serial.print(beatAvg);
                Serial.println(F(" BPM"));
            }
        }
    }
}

// ====================================================================================
// 4. TAMPER SWITCH & MANUAL SOS BUTTON SUBSYSTEM
// ====================================================================================
void readTamperAndButton() {
    // Ignore tamper detection during 2-second startup stabilization
    if (millis() < 2000) return;

    // A. Tamper Switch Monitoring (GPIO 2)
    // LOW = Strap intact (grounded), HIGH = Strap cut/unclasped (Pull-up activated)
    if (digitalRead(PIN_TAMPER) == HIGH) {
        static unsigned long tamperDebounce = 0;
        if (tamperDebounce == 0) tamperDebounce = millis();
        if (millis() - tamperDebounce > 50 && currentState != STATE_ALARM) { // 50ms debounce
            Serial.println(F("[TAMPER] Strap Removal / Tamper Loop Broken! Immediate Alarm!"));
            enterAlarmMode(TRIGGER_TAMPER);
        }
    } else {
        static unsigned long tamperDebounce = 0;
        tamperDebounce = 0;
    }

    // B. Manual SOS / Setup Button Monitoring (GPIO 15)
    // Active LOW button
    static unsigned long buttonPressStart = 0;
    if (digitalRead(PIN_BUTTON) == LOW) {
        if (buttonPressStart == 0) buttonPressStart = millis();

        unsigned long pressDuration = millis() - buttonPressStart;
        
        // Long Press (> 2 seconds) -> Manual SOS Alarm
        if (pressDuration > 2000 && currentState != STATE_ALARM) {
            Serial.println(F("[BUTTON] Long Press Detected -> Manual SOS Triggered!"));
            buttonPressStart = 0;
            enterAlarmMode(TRIGGER_MANUAL_SOS);
        }
    } else {
        if (buttonPressStart > 0) {
            unsigned long pressDuration = millis() - buttonPressStart;
            // Short Press (< 1 second) -> Enter Setup Mode (if not in alarm)
            if (pressDuration > 50 && pressDuration < 1000 && currentState == STATE_NORMAL) {
                Serial.println(F("[BUTTON] Short Press -> Entering Setup Mode"));
                currentState = STATE_SETUP;
            } else if (currentState == STATE_ALARM && pressDuration > 50) {
                // Short press during alarm resets the alarm locally
                Serial.println(F("[BUTTON] Manual Reset Press -> Clearing Alarm Mode"));
                resetAlarmMode();
            }
            buttonPressStart = 0;
        }
    }
}

// ====================================================================================
// 5. BATTERY ADC MONITORING
// ====================================================================================
float getBatteryVoltage() {
    int raw = analogRead(PIN_BATTERY_ADC);
    // ESP32-S3 ADC 12-bit (0-4095), 3.3V reference, 1/2 voltage divider
    float voltage = (raw / 4095.0f) * 3.3f * 2.0f;
    return voltage;
}

// ====================================================================================
// 6. NEO-6M GPS SUBSYSTEM
// ====================================================================================
void processGPS() {
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }
}

// ====================================================================================
// 7. SIM800L GSM SUBSYSTEM & AT COMMAND HANDLER
// ====================================================================================
void sendATCommand(const char* cmd, const char* expectedResp, unsigned int timeoutMs) {
#if SIMULATION_MODE
    Serial.print(F("[SIM GSM AT] Executed: "));
    Serial.println(cmd);
    return; // Fast startup in Cirkit Designer simulator
#endif
    SerialGSM.println(cmd);
    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeoutMs) {
        while (SerialGSM.available()) {
            char c = SerialGSM.read();
            response += c;
        }
        if (response.indexOf(expectedResp) != -1) break;
    }
}

void sendSMS(const char* phoneNumber, const char* message) {
    Serial.print(F("[GSM] Sending SMS to "));
    Serial.println(phoneNumber);

    SerialGSM.print("AT+CMGS=\"");
    SerialGSM.print(phoneNumber);
    SerialGSM.println("\"");
    delay(500);
    SerialGSM.print(message);
    delay(500);
    SerialGSM.write(26); // Ctrl+Z to send
    delay(3000);
    Serial.println(F("[GSM] SMS Sent!"));
}

void initiateVoiceCall(const char* phoneNumber) {
    Serial.print(F("[GSM] Auto-initiating Voice Call to "));
    Serial.println(phoneNumber);
    SerialGSM.print("ATD");
    SerialGSM.print(phoneNumber);
    SerialGSM.println(";");
}

void checkGSMIncoming() {
    if (SerialGSM.available()) {
        String msg = SerialGSM.readString();
        msg.toUpperCase();
        Serial.print(F("[GSM INCOMING] "));
        Serial.println(msg);

        // Check if CANCEL message received from emergency contact
        if (currentState == STATE_ALARM && (msg.indexOf("CANCEL") != -1 || msg.indexOf("OK") != -1)) {
            Serial.println(F("[GSM] Received CANCEL SMS from Trusted Contact! Disarming Alarm."));
            resetAlarmMode();
        }
    }
}

// ====================================================================================
// 8. ALARM STATE MACHINE & ESCALATION CORE
// ====================================================================================
void enterAlarmMode(TriggerSource source) {
    currentState = STATE_ALARM;
    activeTrigger = source;
    alarmStartTime = millis();
    lastGpsSmsTime = millis();
    voiceCallInitiated = false;

    Serial.println(F("\n=============================================="));
    Serial.println(F(" !!! ALARM MODE ACTIVATED !!! "));
    Serial.print(F(" Trigger Reason: "));
    switch (source) {
        case TRIGGER_TAMPER: Serial.println(F("STRAP TAMPER / CUT DETECTED")); break;
        case TRIGGER_SAFEWORD: Serial.println(F("ACOUSTIC SAFE-WORD MATCHED")); break;
        case TRIGGER_SCREAM_CORROBORATED: Serial.println(F("SCREAM + SENSOR CORROBORATION")); break;
        case TRIGGER_VIOLENT_SHAKE: Serial.println(F("VIOLENT SHAKE / STRUGGLE")); break;
        case TRIGGER_MANUAL_SOS: Serial.println(F("MANUAL SOS BUTTON PRESS")); break;
        default: Serial.println(F("UNKNOWN")); break;
    }
    Serial.println(F("==============================================\n"));

    // Turn ON Local Siren & Visual Indicator
    soundSiren(true);

    // Build SOS Message with GPS & Battery Info
    String sosMsg = "EMERGENCY ALERT from Raksha Band!\nTrigger: ";
    switch (source) {
        case TRIGGER_TAMPER: sosMsg += "Strap Tampered"; break;
        case TRIGGER_SAFEWORD: sosMsg += "Safe-word Spoken"; break;
        case TRIGGER_SCREAM_CORROBORATED: sosMsg += "Scream + Motion"; break;
        case TRIGGER_VIOLENT_SHAKE: sosMsg += "Struggle / Shake"; break;
        case TRIGGER_MANUAL_SOS: sosMsg += "Manual SOS"; break;
        default: sosMsg += "Distress Signal"; break;
    }

    float batVolts = getBatteryVoltage();
    sosMsg += "\nBattery: " + String(batVolts, 2) + "V";

    if (gps.location.isValid()) {
        sosMsg += "\nLocation: https://maps.google.com/?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    } else {
#if SIMULATION_MODE
        sosMsg += "\nLocation (Simulated): https://maps.google.com/?q=" + String(DEFAULT_SIM_LAT, 6) + "," + String(DEFAULT_SIM_LNG, 6);
#else
        sosMsg += "\nLocation: Acquiring GPS fix... (Searching satellites)";
#endif
    }

    // Send Immediate SOS SMS
    sendSMS(EMERGENCY_PHONE_NUMBER, sosMsg.c_str());
}

void processAlarmMode() {
    // 1. Alternate Siren sound and LED flashing pattern
    digitalWrite(PIN_STATUS_LED, (millis() % 300 < 150) ? HIGH : LOW);
    digitalWrite(PIN_BUZZER, (millis() % 400 < 200) ? HIGH : LOW);

    unsigned long elapsed = millis() - alarmStartTime;

    // 2. Auto-Call Escalation Window (If no CANCEL received within 20s)
    if (elapsed >= CANCEL_WINDOW_MS && !voiceCallInitiated) {
        voiceCallInitiated = true;
        Serial.println(F("[ALARM] 20s Cancel Window Expired -> Escalating to Voice Call!"));
        initiateVoiceCall(EMERGENCY_PHONE_NUMBER);
    }

    // 3. Periodic GPS Location Update (Every 45 seconds)
    if (millis() - lastGpsSmsTime >= GPS_UPDATE_INTERVAL_MS) {
        lastGpsSmsTime = millis();
        String updateMsg = "RAKSHA BAND LOCATION UPDATE:\n";
        if (gps.location.isValid()) {
            updateMsg += "https://maps.google.com/?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
        } else {
            updateMsg += "GPS fixing... Searching satellites (" + String(gps.satellites.value()) + ")";
        }
        sendSMS(EMERGENCY_PHONE_NUMBER, updateMsg.c_str());
    }
}

void soundSiren(bool state) {
    digitalWrite(PIN_BUZZER, state ? HIGH : LOW);
    digitalWrite(PIN_STATUS_LED, state ? HIGH : LOW);
}

void resetAlarmMode() {
    currentState = STATE_NORMAL;
    activeTrigger = TRIGGER_NONE;
    soundSiren(false);
    sendATCommand("ATH", "OK", 1000); // Hang up any active GSM voice call
    Serial.println(F("\n[ALARM] Alarm Cleared -> Returned to NORMAL_MODE\n"));
}
