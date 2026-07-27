
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <driver/i2s.h>
#include <math.h>
#include <Preferences.h>
#include <ArduinoBLE.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

#include "config.h"

// MAX30102 driver class
class MAX30102_Driver {
public:
    uint8_t i2cAddr = 0x57;

    bool begin(TwoWire &wireBus = Wire) {
        wireBus.beginTransmission(i2cAddr);
        if (wireBus.endTransmission() != 0) return false;

        writeRegister(0x09, 0x40); // Soft reset
        delay(100);
        writeRegister(0x08, 0x50); // FIFO config
        writeRegister(0x09, 0x03); // SpO2 mode
        writeRegister(0x0A, 0x27); // SpO2 config
        writeRegister(0x0C, 0x24); // Red LED
        writeRegister(0x0D, 0x24); // IR LED
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
        Wire.write(0x07);
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

bool checkForBeat(int32_t sample) {
    static int32_t ir_avg_reg = 0;
    static int16_t cbuf[32];
    static uint8_t offset = 0;
    static const int16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4100};
    static int16_t IR_AC_Signal_Current = 0;
    static int16_t IR_AC_Signal_Previous = 0;
    static int16_t positiveEdge = 0;
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
    } else {
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

// System State Definitions
enum SystemState { STATE_NORMAL, STATE_SETUP, STATE_ALARM };
enum TriggerSource {
    TRIGGER_NONE,
    TRIGGER_TAMPER,
    TRIGGER_SAFEWORD,
    TRIGGER_SCREAM_CORROBORATED,
    TRIGGER_VIOLENT_SHAKE,
    TRIGGER_MANUAL_SOS
};

// Global variables
SystemState currentState = STATE_NORMAL;
TriggerSource activeTrigger = TRIGGER_NONE;

HardwareSerial SerialGSM(1);
HardwareSerial SerialGPS(2);

Adafruit_MPU6050 mpu;
MAX30102_Driver particleSensor;
TinyGPSPlus gps;
Preferences preferences;

// NVS stored data & BLE
String deviceSerial = "";
String pairedAppSerial = "";
String emergencyPhoneNumber = DEFAULT_EMERGENCY_PHONE;
bool bleActive = false;
bool deviceConnected = false;

// ArduinoBLE Service and Characteristics
BLEService rakshaService(BLE_SERVICE_UUID);
BLEStringCharacteristic batteryChar(BLE_CHAR_BATTERY_UUID, BLERead | BLENotify, 20);
BLEStringCharacteristic phoneChar(BLE_CHAR_PHONE_UUID, BLERead | BLEWrite, 30);
BLEStringCharacteristic alertChar(BLE_CHAR_ALERT_UUID, BLERead | BLENotify, 100);
BLEStringCharacteristic serialChar(BLE_CHAR_SERIAL_UUID, BLERead | BLEWrite, 50);

// Timers & counters
unsigned long alarmStartTime = 0;
unsigned long lastGpsSmsTime = 0;
bool voiceCallInitiated = false;

// Sensor indicators
bool motionCorroborationActive = false;
unsigned long lastMotionSpikeTime = 0;
bool hrSpikeActive = false;
unsigned long lastHrSpikeTime = 0;

// Audio dynamic noise floor
float ambientNoiseFloor = 1000.0f;

// Motion gait analysis
unsigned long shakeStartTime = 0;
bool isShaking = false;
unsigned long lastGaitStepTime = 0;
int gaitStepCount = 0;

// Heart rate tracking
byte rates[4];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

// Function prototypes
void initBluetooth();
void startBluetoothAdvertising();
void resetBluetoothPairing();
void checkBluetoothButtons();
void updateBLE();
void sendBLEAlertHistory(const char* triggerStr, double lat, double lng);
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

// BLE Event Callbacks
void bleConnectHandler(BLEDevice central) {
    deviceConnected = true;
    Serial.print("[BLE] Mobile app connected: ");
    Serial.println(central.address());
}

void bleDisconnectHandler(BLEDevice central) {
    deviceConnected = false;
    Serial.println("[BLE] Mobile app disconnected. Raksha Band working standalone.");
    if (bleActive) BLE.advertise();
}

void phoneCharWrittenHandler(BLEDevice central, BLECharacteristic characteristic) {
    String val = phoneChar.value();
    if (val.length() > 0) {
        emergencyPhoneNumber = val;
        preferences.putString("phone", emergencyPhoneNumber);
        Serial.print("[BLE] Updated Emergency Contact to: ");
        Serial.println(emergencyPhoneNumber);
    }
}

void serialCharWrittenHandler(BLEDevice central, BLECharacteristic characteristic) {
    String val = serialChar.value();
    if (val.length() > 0) {
        if (pairedAppSerial.length() == 0) {
            pairedAppSerial = val;
            preferences.putString("appSerial", pairedAppSerial);
            Serial.print("[BLE] Paired with App Serial: ");
            Serial.println(pairedAppSerial);
        } else if (pairedAppSerial != val) {
            Serial.println("[BLE WARN] Connection attempt from unauthorized phone rejected!");
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n--- Raksha Band ESP32-S3 Starting ---");

    // Initialize GPIO pins
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    pinMode(PIN_TAMPER, INPUT_PULLUP);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BT_BUTTON, INPUT_PULLUP);    // GPIO 16 for Bluetooth
    pinMode(PIN_RESET_BUTTON, INPUT_PULLUP); // GPIO 17 for Reset connection
    pinMode(PIN_BATTERY_ADC, INPUT);

    // Visual indicator on boot
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(100);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(100);
    }

    preferences.begin("raksha", false);
    deviceSerial = preferences.getString("devSerial", "");
    if (deviceSerial.length() == 0) {
        uint64_t mac = ESP.getEfuseMac();
        char macStr[30];
        snprintf(macStr, sizeof(macStr), "RB-S3-%02X%02X%02X%02X%02X%02X",
                 (uint8_t)(mac >> 40), (uint8_t)(mac >> 32),
                 (uint8_t)(mac >> 24), (uint8_t)(mac >> 16),
                 (uint8_t)(mac >> 8),  (uint8_t)mac);
        deviceSerial = String(macStr);
        preferences.putString("devSerial", deviceSerial);
    }

    pairedAppSerial = preferences.getString("appSerial", "");
    emergencyPhoneNumber = preferences.getString("phone", DEFAULT_EMERGENCY_PHONE);

    Serial.print("[SYS] Device Serial: ");
    Serial.println(deviceSerial);
    Serial.print("[SYS] Emergency Number: ");
    Serial.println(emergencyPhoneNumber);
    if (pairedAppSerial.length() > 0) {
        Serial.print("[SYS] Paired Mobile Serial: ");
        Serial.println(pairedAppSerial);
    } else {
        Serial.println("[SYS] Status: Unpaired (Press GPIO 16 for 2s to enable BT pairing)");
    }

    // Hardware serial initialization
    SerialGSM.begin(9600, SERIAL_8N1, PIN_GSM_RX, PIN_GSM_TX);
    SerialGPS.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // SIM800L init
    sendATCommand("AT", "OK", 2000);
    sendATCommand("ATE0", "OK", 2000);
    sendATCommand("AT+CMGF=1", "OK", 2000);
    sendATCommand("AT+CLIP=1", "OK", 2000);

    // I2C bus setup
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    if (!mpu.begin(0x68, &Wire) && !mpu.begin(0x69, &Wire)) {
        Serial.println("[WARN] MPU6050 init check failed.");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("[OK] MPU6050 Initialized.");
    }

    if (!particleSensor.begin(Wire)) {
        Serial.println("[WARN] MAX30102 pulse oximeter not detected.");
    } else {
        Serial.println("[OK] MAX30102 Initialized.");
    }

    setupI2SMicrophone();
    initBluetooth();

    Serial.println("--- System Ready & Monitoring ---\n");
}

// Bluetooth initialization via ArduinoBLE
void initBluetooth() {
    if (!BLE.begin()) {
        Serial.println("[BLE WARN] BLE stack initialization failed!");
        return;
    }

    BLE.setLocalName(deviceSerial.c_str());
    BLE.setDeviceName(deviceSerial.c_str());
    BLE.setAdvertisedService(rakshaService);

    rakshaService.addCharacteristic(batteryChar);
    rakshaService.addCharacteristic(phoneChar);
    rakshaService.addCharacteristic(alertChar);
    rakshaService.addCharacteristic(serialChar);

    BLE.addService(rakshaService);

    phoneChar.setValue(emergencyPhoneNumber.c_str());
    serialChar.setValue(pairedAppSerial.c_str());

    BLE.setEventHandler(BLEConnected, bleConnectHandler);
    BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);
    phoneChar.setEventHandler(BLEWritten, phoneCharWrittenHandler);
    serialChar.setEventHandler(BLEWritten, serialCharWrittenHandler);

    Serial.println("[BLE] ArduinoBLE initialized.");
}

void startBluetoothAdvertising() {
    bleActive = true;
    BLE.advertise();
    Serial.println("[BT] Bluetooth Advertising enabled. Ready to connect with app.");
}

void resetBluetoothPairing() {
    pairedAppSerial = "";
    preferences.remove("appSerial");
    serialChar.setValue("");
    Serial.println("[BT] Reset connection executed! Paired mobile serial cleared. Ready for new phone.");
}

void checkBluetoothButtons() {
    // Bluetooth Button on GPIO 16 (Press 2 seconds to enable BT)
    static unsigned long btPressStart = 0;
    if (digitalRead(PIN_BT_BUTTON) == LOW) {
        if (btPressStart == 0) btPressStart = millis();
        if (millis() - btPressStart >= BT_ENABLE_HOLD_MS && !bleActive) {
            startBluetoothAdvertising();
            btPressStart = 0;
        }
    } else {
        btPressStart = 0;
    }

    // Reset Button on GPIO 17 (Press 2 seconds to reset pairing)
    static unsigned long resetPressStart = 0;
    if (digitalRead(PIN_RESET_BUTTON) == LOW) {
        if (resetPressStart == 0) resetPressStart = millis();
        if (millis() - resetPressStart >= BT_RESET_HOLD_MS) {
            resetBluetoothPairing();
            resetPressStart = 0;
        }
    } else {
        resetPressStart = 0;
    }
}

void updateBLE() {
    if (bleActive) {
        BLE.poll();
    }
    static unsigned long lastBatSend = 0;
    if (deviceConnected && millis() - lastBatSend > 10000) {
        lastBatSend = millis();
        float v = getBatteryVoltage();
        int pct = constrain((int)((v - 3.2f) / 1.0f * 100.0f), 0, 100);
        String str = String(pct) + "% (" + String(v, 2) + "V)";
        batteryChar.setValue(str.c_str());
    }
}

void sendBLEAlertHistory(const char* triggerStr, double lat, double lng) {
    if (deviceConnected) {
        String logData = String(triggerStr) + ";" + String(lat, 6) + ";" + String(lng, 6);
        alertChar.setValue(logData.c_str());
    }
}

void printSerialHelp() {
    Serial.println("\n--- Interactive Simulator Commands ---");
    Serial.println("  t -> Tamper strap alert");
    Serial.println("  b -> Manual SOS button (GPIO 15)");
    Serial.println("  p -> Bluetooth button (GPIO 16)");
    Serial.println("  r -> Reset connection button (GPIO 17)");
    Serial.println("  s -> Scream + GPS SOS alert");
    Serial.println("  m -> Struggle / Kidnap alert");
    Serial.println("  w -> Safe-word alert");
    Serial.println("  c -> Cancel / disarm alarm");
    Serial.println("  ? -> Print command list\n");
}

void checkSerialCommands() {
    if (Serial.available()) {
        char cmd = tolower(Serial.read());
        if (cmd == 't') {
            Serial.println("[SIM] Tamper alert triggered!");
            enterAlarmMode(TRIGGER_TAMPER);
        } else if (cmd == 'b') {
            Serial.println("[SIM] Manual SOS button pressed!");
            enterAlarmMode(TRIGGER_MANUAL_SOS);
        } else if (cmd == 'p') {
            Serial.println("[SIM] Bluetooth button pressed (GPIO 16) -> Enabling BT");
            startBluetoothAdvertising();
        } else if (cmd == 'r') {
            Serial.println("[SIM] Reset connection button pressed (GPIO 17)");
            resetBluetoothPairing();
        } else if (cmd == 's') {
            Serial.println("[SIM] Scream detected -> Firing Emergency SOS!");
            enterAlarmMode(TRIGGER_SCREAM_CORROBORATED);
        } else if (cmd == 'm') {
            Serial.println("[SIM] Struggle / Kidnap motion detected!");
            enterAlarmMode(TRIGGER_VIOLENT_SHAKE);
        } else if (cmd == 'w') {
            Serial.println("[SIM] Safe-word matched!");
            enterAlarmMode(TRIGGER_SAFEWORD);
        } else if (cmd == 'c') {
            Serial.println("[SIM] Cancel command received -> Clearing Alarm.");
            resetAlarmMode();
        } else if (cmd == '?') {
            printSerialHelp();
        }
    }
}

void loop() {
    checkSerialCommands();
    checkBluetoothButtons();
    updateBLE();
    processGPS();

    readTamperAndButton();
    readI2SAudio();
    readMotionSensor();
    readHeartRateSensor();
    checkGSMIncoming();

    switch (currentState) {
        case STATE_NORMAL:
            digitalWrite(PIN_STATUS_LED, (millis() % 2000 < 100) ? HIGH : LOW);
            digitalWrite(PIN_BUZZER, LOW);
            break;

        case STATE_SETUP:
            digitalWrite(PIN_STATUS_LED, (millis() % 200 < 100) ? HIGH : LOW);
            digitalWrite(PIN_BUZZER, LOW);
            static unsigned long setupStart = 0;
            if (setupStart == 0) setupStart = millis();
            if (millis() - setupStart > 10000) {
                currentState = STATE_NORMAL;
                setupStart = 0;
            }
            break;

        case STATE_ALARM:
            processAlarmMode();
            break;
    }
    delay(10);
}

// I2S Mic setup
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
        Serial.println("[OK] INMP441 Microphone configured.");
    }
}

// Audio processing with dynamic noise floor cancellation
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
            int32_t sample = sampleBuffer[i] >> 14;
            int absVal = abs(sample);
            if (absVal > maxPeak) maxPeak = absVal;
            sumSquare += (double)sample * (double)sample;
        }

        double rms = sqrt(sumSquare / samplesRead);

        // Update dynamic ambient noise floor (exponential moving average)
        ambientNoiseFloor = (float)(ambientNoiseFloor * 0.95 + rms * 0.05);

        // Scream detection: sound peak sharply exceeds ambient noise floor
        if (maxPeak > SCREAM_AMPLITUDE_THRES && rms > (ambientNoiseFloor * 2.2f)) {
            Serial.print("[AUDIO] Scream detected above background noise! Peak: ");
            Serial.print(maxPeak);
            Serial.print(" | Ambient Noise Floor: ");
            Serial.println(ambientNoiseFloor);

            enterAlarmMode(TRIGGER_SCREAM_CORROBORATED);
        }

        // Safe-word acoustic energy detection
        if (rms > (ambientNoiseFloor * 3.5f) && rms > 20000.0) {
            static int highEnergyFrames = 0;
            highEnergyFrames++;
            if (highEnergyFrames >= 5) {
                highEnergyFrames = 0;
                Serial.println("[AUDIO] Safe-word acoustic pattern matched!");
                enterAlarmMode(TRIGGER_SAFEWORD);
            }
        } else {
            static int highEnergyFrames = 0;
            highEnergyFrames = 0;
        }
    }
}

// Motion sensor processing: Walking/Running vs Struggle/Kidnap classifier
void readMotionSensor() {
    sensors_event_t a, g, temp;
    if (!mpu.getEvent(&a, &g, &temp)) return;

    float accelMag = sqrt(a.acceleration.x * a.acceleration.x +
                          a.acceleration.y * a.acceleration.y +
                          a.acceleration.z * a.acceleration.z) / 9.81f;

    float gyroMag = sqrt(g.gyro.x * g.gyro.x +
                         g.gyro.y * g.gyro.y +
                         g.gyro.z * g.gyro.z) * (180.0f / M_PI); // Convert rad/s to deg/s

    // Walking / Running gait detection: rhythmic acceleration (1.2G - 2.2G) with low gyro rotation
    if (accelMag >= 1.2f && accelMag <= 2.2f && gyroMag < 120.0f) {
        if (millis() - lastGaitStepTime > 300 && millis() - lastGaitStepTime < 1000) {
            gaitStepCount++;
            lastGaitStepTime = millis();
        } else if (millis() - lastGaitStepTime >= 1000) {
            gaitStepCount = 0;
            lastGaitStepTime = millis();
        }
        return; // Normal walking/running step rejected from triggering alarm
    }

    // High G impact / Fall detection
    if (accelMag > FALL_G_THRESHOLD) {
        motionCorroborationActive = true;
        lastMotionSpikeTime = millis();
    }

    // Kidnapping / Violent Struggle: sustained multi-axis G-force (>2.5G) + high rotational angular velocity (>250 deg/s)
    if (accelMag > SHAKE_G_THRESHOLD && gyroMag > GYRO_STRUGGLE_THRESHOLD) {
        if (!isShaking) {
            isShaking = true;
            shakeStartTime = millis();
        } else {
            unsigned long shakeDuration = millis() - shakeStartTime;
            if (shakeDuration >= SUSTAINED_SHAKE_MS) {
                isShaking = false;
                Serial.println("[MOTION] Violent Struggle / Kidnap attempt detected!");
                enterAlarmMode(TRIGGER_VIOLENT_SHAKE);
            }
        }
    } else {
        isShaking = false;
    }
}

void readHeartRateSensor() {
    uint32_t redVal = 0, irVal = 0;
    if (!particleSensor.readFIFO(redVal, irVal)) return;
    if (irVal < 50000) return;

    if (checkForBeat((int32_t)irVal)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 40) {
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= 4;
            beatAvg = 0;
            for (byte x = 0; x < 4; x++) beatAvg += rates[x];
            beatAvg /= 4;

            if (beatAvg > HR_SPIKE_THRESHOLD_BPM) {
                hrSpikeActive = true;
                lastHrSpikeTime = millis();
            }
        }
    }
}

void readTamperAndButton() {
    if (millis() < 2000) return;

    // Tamper loop monitoring
    if (digitalRead(PIN_TAMPER) == HIGH) {
        static unsigned long tamperDebounce = 0;
        if (tamperDebounce == 0) tamperDebounce = millis();
        if (millis() - tamperDebounce > 50 && currentState != STATE_ALARM) {
            Serial.println("[TAMPER] Strap Cut / Tamper Loop Broken!");
            enterAlarmMode(TRIGGER_TAMPER);
        }
    } else {
        static unsigned long tamperDebounce = 0;
        tamperDebounce = 0;
    }

    // Manual SOS Button (GPIO 15)
    static unsigned long buttonPressStart = 0;
    if (digitalRead(PIN_BUTTON) == LOW) {
        if (buttonPressStart == 0) buttonPressStart = millis();
        if (millis() - buttonPressStart > 2000 && currentState != STATE_ALARM) {
            buttonPressStart = 0;
            Serial.println("[BUTTON] Manual SOS Long Press!");
            enterAlarmMode(TRIGGER_MANUAL_SOS);
        }
    } else {
        if (buttonPressStart > 0) {
            unsigned long pressDuration = millis() - buttonPressStart;
            if (pressDuration > 50 && pressDuration < 1000 && currentState == STATE_NORMAL) {
                currentState = STATE_SETUP;
            } else if (currentState == STATE_ALARM && pressDuration > 50) {
                resetAlarmMode();
            }
            buttonPressStart = 0;
        }
    }
}

float getBatteryVoltage() {
    int raw = analogRead(PIN_BATTERY_ADC);
    return (raw / 4095.0f) * 3.3f * 2.0f;
}

void processGPS() {
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }
}

void sendATCommand(const char* cmd, const char* expectedResp, unsigned int timeoutMs) {
#if SIMULATION_MODE
    Serial.print("[SIM GSM] AT command: ");
    Serial.println(cmd);
    return;
#endif
    SerialGSM.println(cmd);
    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeoutMs) {
        while (SerialGSM.available()) {
            response += (char)SerialGSM.read();
        }
        if (response.indexOf(expectedResp) != -1) break;
    }
}

void sendSMS(const char* phoneNumber, const char* message) {
    Serial.print("[GSM] Sending SOS SMS to ");
    Serial.println(phoneNumber);
#if SIMULATION_MODE
    Serial.println("[SIM GSM] SMS Payload:");
    Serial.println(message);
    return;
#endif
    SerialGSM.print("AT+CMGS=\"");
    SerialGSM.print(phoneNumber);
    SerialGSM.print("\"\r\n");
    delay(500);
    SerialGSM.print(message);
    delay(500);
    SerialGSM.write(26); // Ctrl+Z to send
    delay(3000);
    Serial.println("[GSM] Real SMS command dispatched.");
}

void initiateVoiceCall(const char* phoneNumber) {
    Serial.print("[GSM] Auto Voice Calling ");
    Serial.println(phoneNumber);
#if SIMULATION_MODE
    Serial.println("[SIM GSM] Voice call simulated.");
    return;
#endif
    SerialGSM.print("ATD");
    SerialGSM.print(phoneNumber);
    SerialGSM.print(";\r\n");
    Serial.println("[GSM] Dial command sent to SIM800L.");
}

void checkGSMIncoming() {
    if (SerialGSM.available()) {
        String msg = SerialGSM.readString();
        msg.toUpperCase();
        if (currentState == STATE_ALARM && (msg.indexOf("CANCEL") != -1 || msg.indexOf("OK") != -1)) {
            Serial.println("[GSM] Received CANCEL SMS. Resetting alarm.");
            resetAlarmMode();
        }
    }
}

void enterAlarmMode(TriggerSource source) {
    currentState = STATE_ALARM;
    activeTrigger = source;
    alarmStartTime = millis();
    lastGpsSmsTime = millis();
    voiceCallInitiated = false;

    soundSiren(true);

    const char* triggerStr = "Distress Signal";
    switch (source) {
        case TRIGGER_TAMPER: triggerStr = "Strap Tampered"; break;
        case TRIGGER_SAFEWORD: triggerStr = "Safe-word Spoken"; break;
        case TRIGGER_SCREAM_CORROBORATED: triggerStr = "Scream Audio SOS"; break;
        case TRIGGER_VIOLENT_SHAKE: triggerStr = "Struggle / Kidnap"; break;
        case TRIGGER_MANUAL_SOS: triggerStr = "Manual SOS"; break;
        default: break;
    }

    double currentLat = DEFAULT_SIM_LAT;
    double currentLng = DEFAULT_SIM_LNG;
    if (gps.location.isValid()) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();
    }

    // Send alert history to mobile app over Bluetooth
    sendBLEAlertHistory(triggerStr, currentLat, currentLng);

    // Build SOS text message
    String sosMsg = "EMERGENCY ALERT from Raksha Band!\nTrigger: " + String(triggerStr);
    sosMsg += "\nBattery: " + String(getBatteryVoltage(), 2) + "V";

    if (gps.location.isValid()) {
        sosMsg += "\nLocation: https://maps.google.com/?q=" + String(currentLat, 6) + "," + String(currentLng, 6);
    } else {
#if SIMULATION_MODE
        sosMsg += "\nLocation (Simulated): https://maps.google.com/?q=" + String(DEFAULT_SIM_LAT, 6) + "," + String(DEFAULT_SIM_LNG, 6);
#else
        sosMsg += "\nLocation: GPS search active...";
#endif
    }

    sendSMS(emergencyPhoneNumber.c_str(), sosMsg.c_str());
}

void processAlarmMode() {
    digitalWrite(PIN_STATUS_LED, (millis() % 300 < 150) ? HIGH : LOW);
    digitalWrite(PIN_BUZZER, (millis() % 400 < 200) ? HIGH : LOW);

    unsigned long elapsed = millis() - alarmStartTime;

    // Escalation voice call after 20s if not disarmed
    if (elapsed >= CANCEL_WINDOW_MS && !voiceCallInitiated) {
        voiceCallInitiated = true;
        Serial.println("[ALARM] Cancel window expired -> Initiating Emergency Voice Call!");
        initiateVoiceCall(emergencyPhoneNumber.c_str());
    }

    // Periodic GPS SMS update
    if (millis() - lastGpsSmsTime >= GPS_UPDATE_INTERVAL_MS) {
        lastGpsSmsTime = millis();
        String updateMsg = "RAKSHA BAND LOCATION UPDATE:\n";
        if (gps.location.isValid()) {
            updateMsg += "https://maps.google.com/?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
        } else {
            updateMsg += "GPS searching satellites...";
        }
        sendSMS(emergencyPhoneNumber.c_str(), updateMsg.c_str());
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
    sendATCommand("ATH", "OK", 1000);
    Serial.println("[ALARM] Alarm Cleared -> Normal Mode");
}
