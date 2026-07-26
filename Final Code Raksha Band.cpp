#include <cmath>
#include <cstring>
#include <cctype>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Safety_Bracelet_inferencing.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLESecurity.h>
#include <Update.h>
#include <driver/i2s.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <mbedtls/md.h>

// --- CONFIGURATION CONSTANTS ---
constexpr uint8_t STATUS_LED_PIN = 21;
constexpr uint8_t BUZZER_PIN = 3;
constexpr uint8_t TAMPER_PIN = 2;
constexpr uint8_t MANUAL_SOS_PIN = 15;
constexpr uint8_t VBAT_PIN = 1;
constexpr uint8_t SIM800L_TX = 4;
constexpr uint8_t SIM800L_RX = 5;
constexpr uint8_t GPS_TX = 6;
constexpr uint8_t GPS_RX = 7;

constexpr uint8_t I2S_WS = 42;
constexpr uint8_t I2S_SD = 41;
constexpr uint8_t I2S_SCK = 43;
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

constexpr uint8_t MAX_CONTACTS = 3;
constexpr uint8_t WDT_TIMEOUT_SECONDS = 10;
constexpr int LOG_MAX_RECORDS = 20;

constexpr float ACCEL_G_THRESHOLD = 2.5f;
constexpr float AI_CONFIDENCE_THRESHOLD = 0.85f;
constexpr int MIN_BPM = 40;
constexpr int MAX_BPM = 200;
constexpr int HIGH_HR_THRESHOLD = 110;
constexpr unsigned long GPS_STALE_TIMEOUT = 5000;
constexpr unsigned long MANUAL_SOS_PRESS_MS = 3000;
constexpr unsigned long SOS_COOLDOWN_MS = 300000;
constexpr uint8_t MAX_RETRY_LIMIT = 3;
constexpr long DEFAULT_IR_THRESHOLD = 50000;
constexpr long MIN_IR_THRESH = 10000;  
constexpr long MAX_IR_THRESH = 100000; 

constexpr unsigned long GSM_REG_TIMEOUT = 3000; 
constexpr unsigned long GSM_PROMPT_TIMEOUT = 3000; 
constexpr unsigned long GSM_RESULT_TIMEOUT = 6000; 

#define DEVICE_ID "BRACELET_ESP32_01"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- SYSTEM STATES ---
enum DeviceState { STATE_INIT, STATE_MONITORING, STATE_POSSIBLE_SCREAM, STATE_VERIFYING, STATE_SOS_TRIGGERED, STATE_COOLDOWN, STATE_ERROR };
DeviceState currentState = STATE_INIT;

enum GsmState { GSM_IDLE, GSM_CHECK_REG, GSM_WAIT_REG, GSM_SETUP_SMS, GSM_WAIT_PROMPT, GSM_SEND_PAYLOAD, GSM_WAIT_RESULT };
GsmState smsState = GSM_IDLE;

// --- GLOBAL VARIABLES ---
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;
TinyGPSPlus gps;
HardwareSerial sim800l(1);
HardwareSerial gpsSerial(2);
Preferences preferences;

SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t stateMutex = NULL;

char emergencyContacts[MAX_CONTACTS][20];
int activeContactCount = 0;
long irFingerThreshold = DEFAULT_IR_THRESHOLD;

float currentBPM = 0;
float bpmReadings[8] = {0};
int bpmIndex = 0;
unsigned long lastBeatTime = 0;

float ai_features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
int16_t sample_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]; 
float confidenceHistory[10] = {0};
int confIndex = 0;
int screamLabelIndex = -1;

unsigned long stateTimer = 0;
unsigned long sosTriggerTime = 0;
unsigned long manualButtonPressTime = 0;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

char lastTriggerReason[32] = "None";
char liveGPSLink[128] = "Location_Unavailable_Stale"; 
char diagnosticStatusStr[128] = "System Initializing";
char otaHmacSecret[64]; 

bool globalMpuOk = false;
bool globalMaxOk = false;
bool otaAuthenticated = false;

uint32_t currentOtaSession = 0;
uint32_t expectedOtaChunk = 0;

unsigned long buzzerMillis = 0;
int buzzerState = LOW;
int buzzerPatternType = 0;
unsigned long errorLoopMillis = 0;

unsigned long gsmTimer = 0;
uint8_t gsmRetryCount = 0;
uint8_t currentSmsIndex = 0;
char gsmPayload[160];

// --- THREAD-SAFE STATE ACCESSORS ---
void setDeviceState(DeviceState newState) {
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        currentState = newState;
        xSemaphoreGive(stateMutex);
    }
}

DeviceState getDeviceState() {
    DeviceState s = STATE_ERROR;
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        s = currentState;
        xSemaphoreGive(stateMutex);
    }
    return s;
}

int getActiveContactCount() {
    int count = 0;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        count = activeContactCount;
        xSemaphoreGive(dataMutex);
    }
    return count;
}

// --- ADC CALIBRATION ---
int getBatteryPercentage() {
    long sum = 0;
    for (int i = 0; i < 10; i++) { sum += analogRead(VBAT_PIN); delay(2); }
    float rawADC = (float)sum / 10.0f; 
    float v = (rawADC / 4095.0f) * 3.3f * 2.0f * 1.02f;
    const float voltages[] = {4.20, 4.10, 4.00, 3.90, 3.80, 3.70, 3.60, 3.50, 3.40, 3.30};
    const int percents[]   = {100,  90,   80,   70,   60,   45,   30,   15,   5,    0};
    if (v >= voltages[0]) return 100;
    if (v <= voltages[9]) return 0;
    for (int i = 0; i < 9; i++) {
        if (v <= voltages[i] && v >= voltages[i+1]) {
            return percents[i+1] + (int)(((percents[i] - percents[i+1]) * (voltages[i] - v)) / (voltages[i] - voltages[i+1]));
        }
    }
    return 50;
}

// --- SECURE NVS SECRET LOADING ---
void loadSecureSecret() {
    if (preferences.getString("sec_key", otaHmacSecret, sizeof(otaHmacSecret)) == 0) {
        // Enforce removal of default secret[cite: 11]. Fails securely if undefined.
        memset(otaHmacSecret, 0, sizeof(otaHmacSecret)); 
    }
}

// --- I2S INITIALIZATION & RECOVERY ---
void initI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 1024,
        .use_apll = false
    };
    i2s_pin_config_t pin_config = { .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS, .data_out_num = -1, .data_in_num = I2S_SD };
    
    if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK || i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
        setDeviceState(STATE_ERROR);
    }
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) out_ptr[i] = (float)sample_buffer[offset + i] / 32768.0f;
    return 0;
}

// --- PERSISTENT LOGGING WITH ERROR CHECKING ---
void logEventPersistent(const char* reason, float confidence, bool fallRes) {
    float localBPM = 0;
    char localLink[128] = "Unknown";
    
    // Copy shared variables securely into local buffers[cite: 11]
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        localBPM = currentBPM;
        strcpy(localLink, liveGPSLink);
        xSemaphoreGive(dataMutex);
    }
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int logIndex = preferences.getInt("log_idx", 0);
        char key[24]; snprintf(key, sizeof(key), "log_%d", logIndex);
        
        char record[128];
        snprintf(record, sizeof(record), "TS:%lu|R:%s|B:%d|H:%.1f|C:%.2f|F:%d|GPS:%s", 
                 millis(), reason, getBatteryPercentage(), localBPM, confidence, fallRes, localLink);
        
        if (preferences.putString(key, record) > 0) {
            logIndex = (logIndex + 1) % LOG_MAX_RECORDS;
            preferences.putInt("log_idx", logIndex);
        }
        xSemaphoreGive(dataMutex);
    }
}

// --- BLE APP CALLBACKS & OTA PROTECTION ---
bool verifyHMAC(const std::string& payload, const std::string& receivedHashHex) {
    if (strlen(otaHmacSecret) == 0) return false;
    byte mac[32]; mbedtls_md_context_t ctx; mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    mbedtls_md_init(&ctx); mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    mbedtls_md_hmac_starts(&ctx, (const unsigned char *) otaHmacSecret, strlen(otaHmacSecret));
    mbedtls_md_hmac_update(&ctx, (const unsigned char *) payload.c_str(), payload.length());
    mbedtls_md_hmac_finish(&ctx, mac); mbedtls_md_free(&ctx);

    char calcHashHex[65];
    for (int i = 0; i < 32; i++) snprintf(&calcHashHex[i * 2], 3, "%02x", mac[i]);
    return (receivedHashHex == std::string(calcHashHex));
}

void resetOtaSession(bool failed) {
    otaAuthenticated = false;
    currentOtaSession = 0;
    expectedOtaChunk = 0;
    if (failed) logEventPersistent("OTA Update Failed", 0.0f, false); // Log failures[cite: 11]
}

class AppDataCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.size() < 3) return; 
        
        uint8_t msgType = rxValue[0];
        uint8_t length = rxValue[1];
        if (2 + length + 1 > rxValue.size()) return;
        
        uint8_t crc = 0; for (size_t i = 0; i < 2 + length; i++) crc ^= rxValue[i];
        if (rxValue[2 + length] != crc) return;
        
        std::string payload = rxValue.substr(2, length);
        
        if (msgType == 0x01) { 
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (activeContactCount < MAX_CONTACTS) {
                    payload.copy(emergencyContacts[activeContactCount], 19, 0);
                    emergencyContacts[activeContactCount][19] = '\0';
                    char key[16]; snprintf(key, sizeof(key), "phone_%d", activeContactCount);
                    if (preferences.putString(key, emergencyContacts[activeContactCount]) > 0) activeContactCount++; 
                }
                xSemaphoreGive(dataMutex);
            }
        } else if (msgType == 0x03) { 
            char* endPtr;
            long val = strtol(payload.c_str(), &endPtr, 10);
            if (*endPtr == '\0' && val >= MIN_IR_THRESH && val <= MAX_IR_THRESH) { 
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    irFingerThreshold = val;
                    preferences.putLong("ir_thresh", irFingerThreshold);
                    xSemaphoreGive(dataMutex);
                }
            }
        } else if (msgType == 0x05) { 
            setDeviceState(STATE_SOS_TRIGGERED);
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                strcpy(lastTriggerReason, "App Manual SOS"); // Protect shared string write[cite: 11]
                xSemaphoreGive(dataMutex);
            }
        } else if (msgType == 0x06) { 
            if (payload.length() < 76) return;
            
            uint32_t sessionID = 0, chunkNum = 0, totalChunks = 0;
            memcpy(&sessionID, payload.data(), 4);
            memcpy(&chunkNum, payload.data() + 4, 4);
            memcpy(&totalChunks, payload.data() + 8, 4);
            std::string hash = payload.substr(12, 64);
            std::string chunkData = payload.substr(76);

            // Strict Sequencing enforcement[cite: 11]
            if (chunkNum != expectedOtaChunk) return; 
            if (!verifyHMAC(chunkData, hash)) return;
            
            currentOtaSession = sessionID;
            expectedOtaChunk = chunkNum + 1;

            if (!Update.isRunning() && !Update.begin(UPDATE_SIZE_UNKNOWN)) return;
            
            esp_task_wdt_reset(); 
            
            if (Update.write((uint8_t*)chunkData.data(), chunkData.length()) != chunkData.length()) {
                Update.abort(); 
                resetOtaSession(true); // Clear on abort[cite: 11]
            } else if (chunkData.length() == 0 || expectedOtaChunk >= totalChunks) {
                if (Update.end(true) && !Update.hasError()) {
                    resetOtaSession(false); // Clear on success[cite: 11]
                    ESP.restart(); 
                } else {
                    Update.abort();
                    resetOtaSession(true);
                }
            }
        }
    }
};

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    sim800l.begin(9600, SERIAL_8N1, SIM800L_RX, SIM800L_TX);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(TAMPER_PIN, INPUT_PULLUP);
    pinMode(MANUAL_SOS_PIN, INPUT_PULLUP);

    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    dataMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL || stateMutex == NULL) {
        Serial.println("CRITICAL: Mutex initialization failed. Halting system.");
        abort(); 
    }

    preferences.begin("safety_app", false);
    loadSecureSecret(); 
    
    long savedIr = preferences.getLong("ir_thresh", DEFAULT_IR_THRESHOLD);
    irFingerThreshold = (savedIr >= MIN_IR_THRESH && savedIr <= MAX_IR_THRESH) ? savedIr : DEFAULT_IR_THRESHOLD; 
    
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MAX_CONTACTS; i++) {
            char key[16]; snprintf(key, sizeof(key), "phone_%d", i);
            String num = preferences.getString(key, "");
            if (num.length() > 0) { num.toCharArray(emergencyContacts[i], 20); activeContactCount++; }
        }
        xSemaphoreGive(dataMutex);
    }

    BLEDevice::init("Safety_Bracelet");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    
    // Require Authenticated/Encrypted connections for characteristic modification
    pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
    pCharacteristic->setCallbacks(new AppDataCallback());
    pService->start();
    
    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    
    BLEDevice::startAdvertising();

    Wire.begin();
    globalMaxOk = particleSensor.begin(Wire, I2C_SPEED_FAST);
    if (!globalMaxOk) setDeviceState(STATE_ERROR); else particleSensor.setup();
    globalMpuOk = mpu.begin();
    if (!globalMpuOk) setDeviceState(STATE_ERROR);

    initI2S();

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (strcmp(ei_classifier_inferencing_categories[i], "scream") == 0) { screamLabelIndex = i; break; }
    }

    bool gsmOk = true;
    sim800l.println("AT"); delay(200);
    if (!sim800l.find("OK")) gsmOk = false;
    snprintf(diagnosticStatusStr, sizeof(diagnosticStatusStr), "MPU:%d|MAX:%d|GSM:%d|BAT:%d%%", globalMpuOk, globalMaxOk, gsmOk, getBatteryPercentage());
    if (getDeviceState() != STATE_ERROR) setDeviceState((!globalMpuOk || !globalMaxOk || !gsmOk) ? STATE_ERROR : STATE_MONITORING);
}

// --- MAIN LOOP ---
void loop() {
    DeviceState localState = getDeviceState();

    if (buzzerPatternType == 1 && millis() - buzzerMillis >= 150) {
        buzzerMillis = millis(); buzzerState = !buzzerState; digitalWrite(BUZZER_PIN, buzzerState);
    } else if (buzzerPatternType == 2 && millis() - buzzerMillis >= 200) {
        buzzerMillis = millis(); buzzerState = !buzzerState; digitalWrite(BUZZER_PIN, buzzerState);
    } else if (buzzerPatternType == 0) { digitalWrite(BUZZER_PIN, LOW); }

    // Protect liveGPSLink String generation[cite: 11]
    while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
    if (gps.location.isValid() && gps.location.age() < GPS_STALE_TIMEOUT) {
        if (gps.satellites.value() >= 3 && gps.hdop.isValid() && gps.hdop.value() <= 300) {
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                snprintf(liveGPSLink, sizeof(liveGPSLink), "http://maps.google.com/maps?q=%.6f,%.6f", gps.location.lat(), gps.location.lng());
                xSemaphoreGive(dataMutex);
            }
        }
    } else {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            snprintf(liveGPSLink, sizeof(liveGPSLink), "Location_Unavailable_Stale");
            xSemaphoreGive(dataMutex);
        }
    }

    int reading = digitalRead(MANUAL_SOS_PIN);
    if (reading != lastButtonState) lastDebounceTime = millis();
    if ((millis() - lastDebounceTime) > 50) {
        if (reading == LOW) {
            if (manualButtonPressTime == 0) manualButtonPressTime = millis();
            else if (millis() - manualButtonPressTime >= MANUAL_SOS_PRESS_MS) {
                if (localState != STATE_SOS_TRIGGERED && localState != STATE_COOLDOWN) {
                    setDeviceState(STATE_SOS_TRIGGERED); 
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { strcpy(lastTriggerReason, "Manual Long-Press SOS"); xSemaphoreGive(dataMutex); } // Mutex lock[cite: 11]
                }
            }
        } else { manualButtonPressTime = 0; }
    }
    lastButtonState = reading;

    if (digitalRead(TAMPER_PIN) == LOW && localState != STATE_SOS_TRIGGERED && localState != STATE_COOLDOWN) {
        setDeviceState(STATE_SOS_TRIGGERED); 
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { strcpy(lastTriggerReason, "Tamper Switch Triggered"); xSemaphoreGive(dataMutex); } // Mutex lock[cite: 11]
    }

    if (localState == STATE_MONITORING) {
        long irValue = particleSensor.getIR();
        long currentThresh = irFingerThreshold;
        if (xSemaphoreTake(dataMutex, 0) == pdTRUE) { currentThresh = irFingerThreshold; xSemaphoreGive(dataMutex); }
        
        if (irValue >= currentThresh) {
            if (checkForBeat(irValue)) {
                unsigned long currentBeat = millis();
                if (lastBeatTime != 0) {
                    float bpm = 60000.0f / (currentBeat - lastBeatTime);
                    if (bpm >= MIN_BPM && bpm <= MAX_BPM) {
                        bpmReadings[bpmIndex] = bpm; bpmIndex = (bpmIndex + 1) % 8;
                        float sum = 0; int valid = 0;
                        for (int i = 0; i < 8; i++) { if (bpmReadings[i] > 0) { sum += bpmReadings[i]; valid++; } }
                        
                        // Protect HR calculation writing[cite: 11]
                        if (valid > 0) {
                            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                currentBPM = sum / valid;
                                xSemaphoreGive(dataMutex);
                            }
                        }
                    }
                }
                lastBeatTime = currentBeat;
            }
        } else { 
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { currentBPM = 0; xSemaphoreGive(dataMutex); }
        }

        size_t bytes_read = 0;
        esp_err_t err = i2s_read(I2S_PORT, &sample_buffer, sizeof(sample_buffer), &bytes_read, pdMS_TO_TICKS(10));
        
        if (err != ESP_OK || bytes_read != sizeof(sample_buffer)) {
            i2s_driver_uninstall(I2S_PORT); initI2S(); 
        } else {
            signal_t live_audio_signal;
            live_audio_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
            live_audio_signal.get_data = &raw_feature_get_data;

            ei_impulse_result_t result = { 0 };
            if (run_classifier(&live_audio_signal, &result, false) == EI_IMPULSE_OK) {
                float screamConf = (screamLabelIndex != -1) ? result.classification[screamLabelIndex].value : 0.0f;
                
                // Protect Model Matrix confidence records[cite: 11]
                int highConfCount = 0;
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    confidenceHistory[confIndex] = screamConf; confIndex = (confIndex + 1) % 10;
                    for (int i = 0; i < 10; i++) { if (confidenceHistory[i] > AI_CONFIDENCE_THRESHOLD) highConfCount++; }
                    xSemaphoreGive(dataMutex);
                }

                if (highConfCount >= 7) { setDeviceState(STATE_POSSIBLE_SCREAM); stateTimer = millis(); }
            }
        }
    } 
    else if (localState == STATE_POSSIBLE_SCREAM) {
        if (millis() - stateTimer >= 2000) setDeviceState(STATE_VERIFYING);
    } 
    else if (localState == STATE_VERIFYING) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        float totalG = sqrt((a.acceleration.x * a.acceleration.x) + (a.acceleration.y * a.acceleration.y) + (a.acceleration.z * a.acceleration.z)) / 9.81f;

        // Retrieve local BPM securely
        float localBPM = 0;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { localBPM = currentBPM; xSemaphoreGive(dataMutex); }

        if (localBPM > HIGH_HR_THRESHOLD || totalG > ACCEL_G_THRESHOLD) {
            setDeviceState(STATE_SOS_TRIGGERED); 
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { strcpy(lastTriggerReason, "AI Scream & Sensor Verification"); xSemaphoreGive(dataMutex); } // Mutex Lock[cite: 11]
        } else { setDeviceState(STATE_MONITORING); }
    }
    else if (localState == STATE_SOS_TRIGGERED) {
        buzzerPatternType = 2;
        
        char localReason[32] = "Unknown";
        char localLink[128] = "Unknown";
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            strcpy(localReason, lastTriggerReason);
            strcpy(localLink, liveGPSLink);
            xSemaphoreGive(dataMutex);
        }

        logEventPersistent(localReason, 0.9f, false);
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            snprintf(gsmPayload, sizeof(gsmPayload), "EMERGENCY! ID:%s Reason:%s Loc:%s", DEVICE_ID, localReason, localLink);
            xSemaphoreGive(dataMutex);
        }
        
        sim800l.println("ATD181;");
        
        smsState = GSM_CHECK_REG;
        currentSmsIndex = 0;
        gsmRetryCount = 0;
        gsmTimer = millis();
        
        sosTriggerTime = millis();
        setDeviceState(STATE_COOLDOWN);
    }
    else if (localState == STATE_COOLDOWN) {
        int activeContactsLocal = getActiveContactCount(); // Mutex-protected getter[cite: 11]
        
        if (smsState != GSM_IDLE && currentSmsIndex < activeContactsLocal) {
            char contactNum[20];
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { strcpy(contactNum, emergencyContacts[currentSmsIndex]); xSemaphoreGive(dataMutex); }
            
            char simResponse[128] = {0};
            size_t bytesRead = 0;

            switch(smsState) {
                case GSM_CHECK_REG:
                    sim800l.println("AT+CREG?");
                    gsmTimer = millis();
                    smsState = GSM_WAIT_REG;
                    break;
                case GSM_WAIT_REG:
                    if (sim800l.available()) {
                        bytesRead = sim800l.readBytes(simResponse, sizeof(simResponse) - 1);
                        simResponse[bytesRead] = '\0';
                        if (strstr(simResponse, "+CREG: 0,1") != NULL || strstr(simResponse, "+CREG: 0,5") != NULL) smsState = GSM_SETUP_SMS;
                        else if (millis() - gsmTimer > GSM_REG_TIMEOUT) smsState = GSM_CHECK_REG;
                    } else if (millis() - gsmTimer > GSM_REG_TIMEOUT) smsState = GSM_CHECK_REG;
                    break;
                case GSM_SETUP_SMS:
                    sim800l.println("AT+CMGF=1");
                    delay(50); 
                    sim800l.printf("AT+CMGS=\"%s\"\r\n", contactNum);
                    gsmTimer = millis();
                    smsState = GSM_WAIT_PROMPT;
                    break;
                case GSM_WAIT_PROMPT:
                    if (sim800l.available() && sim800l.read() == '>') {
                        smsState = GSM_SEND_PAYLOAD;
                    } else if (millis() - gsmTimer > GSM_PROMPT_TIMEOUT) {
                        gsmRetryCount++;
                        smsState = (gsmRetryCount > MAX_RETRY_LIMIT) ? GSM_IDLE : GSM_SETUP_SMS;
                    }
                    break;
                case GSM_SEND_PAYLOAD:
                    {
                        // Safely retrieve SMS array for network transport
                        char localPayload[160];
                        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                            strcpy(localPayload, gsmPayload);
                            xSemaphoreGive(dataMutex);
                        }
                        sim800l.print(localPayload);
                        sim800l.write(26);
                        gsmTimer = millis();
                        smsState = GSM_WAIT_RESULT;
                    }
                    break;
                case GSM_WAIT_RESULT:
                    if (sim800l.available()) {
                        bytesRead = sim800l.readBytes(simResponse, sizeof(simResponse) - 1);
                        simResponse[bytesRead] = '\0';
                        if (strstr(simResponse, "+CMGS:") != NULL || strstr(simResponse, "OK") != NULL) {
                            currentSmsIndex++; gsmRetryCount = 0;
                            smsState = (currentSmsIndex >= activeContactsLocal) ? GSM_IDLE : GSM_SETUP_SMS;
                        } else if (strstr(simResponse, "ERROR") != NULL) {
                            gsmRetryCount++;
                            if (gsmRetryCount > MAX_RETRY_LIMIT) { currentSmsIndex++; gsmRetryCount = 0; }
                            smsState = (currentSmsIndex >= activeContactsLocal) ? GSM_IDLE : GSM_SETUP_SMS;
                        }
                    } else if (millis() - gsmTimer > GSM_RESULT_TIMEOUT) {
                        gsmRetryCount++;
                        if (gsmRetryCount > MAX_RETRY_LIMIT) { currentSmsIndex++; gsmRetryCount = 0; }
                        smsState = (currentSmsIndex >= activeContactsLocal) ? GSM_IDLE : GSM_SETUP_SMS;
                    }
                    break;
                default: break;
            }
        }
        
        if (millis() - sosTriggerTime > SOS_COOLDOWN_MS) { 
            buzzerPatternType = 0; 
            
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memset(confidenceHistory, 0, sizeof(confidenceHistory));
                confIndex = 0;
                xSemaphoreGive(dataMutex);
            }
            
            setDeviceState(STATE_MONITORING); 
        }
    }
    else if (localState == STATE_ERROR) {
        buzzerPatternType = 1;
        if (millis() - errorLoopMillis >= 2000) errorLoopMillis = millis();
    }

    esp_task_wdt_reset();
}