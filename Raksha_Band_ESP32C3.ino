//Raksha Band Code

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include "config.h"

// Multi-stack BLE Header Auto-Detection using __has_include
#if ENABLE_BLE
  #if __has_include(<NimBLEDevice.h>)
    #include <NimBLEDevice.h>
    #define USE_NIMBLE_LIB 1
    #define HAVE_BLE_STACK 1
  #elif __has_include("esp_gap_ble_api.h")
    #include "esp_bt.h"
    #include "esp_gap_ble_api.h"
    #include "esp_gatts_api.h"
    #if __has_include("esp_bt_main.h")
      #include "esp_bt_main.h"
    #endif
    #define USE_IDF_BLE 1
    #define HAVE_BLE_STACK 1
  #else
    #define HAVE_BLE_STACK 0
  #endif
#else
  #define HAVE_BLE_STACK 0
#endif

// System State Machine
enum BandState {
    STATE_IDLE,
    STATE_COUNTDOWN,
    STATE_DISPATCH,
    STATE_ALARM_ACTIVE
};

// Global variables
BandState g_state = STATE_IDLE;
HardwareSerial SerialSIM(1);
Preferences g_prefs;

String g_emergencyPhone = DEFAULT_PHONE_NUMBER;
String g_deviceSerial = "";
bool g_bleActive = false;
bool g_bleConnected = false;
bool g_bleInitDone = false;
unsigned long g_countdownStart = 0;
unsigned long g_lastMotionSample = 0;

// Motion buffers
float g_accelBuf[16];
uint8_t g_bufIdx = 0;
bool g_lis3dhFound = false;
uint8_t g_lis3dhAddr = LIS3DH_ADDR_PRIMARY;

#if HAVE_BLE_STACK && defined(USE_IDF_BLE)
static esp_ble_adv_params_t g_advParams = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHANNEL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t g_advData = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,
    .p_service_uuid      = NULL,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif

// Forward Declarations
void initHardware();
void startBLE();
void stopBLE();
void initLIS3DH();
void readLIS3DH(float &ax, float &ay, float &az);
void checkMotionEvents();
void checkInputs();
void handleStateCountdown();
void handleStateDispatch();
void handleStateAlarmActive();
void triggerEmergency(String reason);
void cancelEmergency();
bool powerOnModem();
String sendAT(String cmd, unsigned long timeoutMs = 2000);
bool fetchGNSSLocation(float &lat, float &lng);
bool sendSMS(String number, String message);
uint8_t readBatteryLevel();
void enterDeepSleep();

#if HAVE_BLE_STACK && defined(USE_IDF_BLE)
static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            if (g_bleActive) {
                esp_ble_gap_start_advertising(&g_advParams);
            }
            break;
        default:
            break;
    }
}

static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_CONNECT_EVT:
            g_bleConnected = true;
            Serial.println("App Connected via Native BLE!");
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            g_bleConnected = false;
            Serial.println("App Disconnected from Native BLE.");
            if (g_bleActive) {
                esp_ble_gap_start_advertising(&g_advParams);
            }
            break;
        case ESP_GATTS_WRITE_EVT:
            if (param->write.len > 0) {
                char buf[32] = {0};
                uint16_t len = param->write.len < 31 ? param->write.len : 31;
                memcpy(buf, param->write.value, len);
                String val = String(buf);
                val.trim();
                if (val.length() >= 8 && val.startsWith("+")) {
                    g_emergencyPhone = val;
                    g_prefs.putString("phone", g_emergencyPhone);
                    Serial.print("Emergency phone updated via BLE: ");
                    Serial.println(g_emergencyPhone);
                } else if (val == "TRIGGER") {
                    triggerEmergency("App Remote SOS Command");
                } else if (val == "CANCEL") {
                    cancelEmergency();
                }
            }
            break;
        default:
            break;
    }
}
#endif

void setup() {
    Serial.begin(DEBUG_BAUD);
    delay(500);
    Serial.println("\n=============================================");
    Serial.println("   RAKSHA BAND - ESP32-C3 SAFETY FIRMWARE    ");
    Serial.println("=============================================");

    initHardware();

    // Load NVS config
    g_prefs.begin(NVS_NAMESPACE, false);
    g_emergencyPhone = g_prefs.getString("phone", DEFAULT_PHONE_NUMBER);

    // Generate unique device serial from MAC
    uint64_t mac = ESP.getEfuseMac();
    char serialStr[20];
    snprintf(serialStr, sizeof(serialStr), "RAKSHA-C3-%04X", (uint16_t)(mac & 0xFFFF));
    g_deviceSerial = String(serialStr);

    Serial.print("Device Serial : ");
    Serial.println(g_deviceSerial);
    Serial.print("Emergency No. : ");
    Serial.println(g_emergencyPhone);

    initLIS3DH();

    // Check BT switch state on boot
    if (digitalRead(PIN_BT_SWITCH) == LOW) {
        startBLE();
    } else {
        Serial.println("Bluetooth Switch is OFF. (Flip switch on GPIO 0 to turn ON)");
    }

    // Check wake cause
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO || wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println("Woken from deep sleep via Hardware Interrupt!");
        triggerEmergency("Wake Interrupt (Motion/Reed)");
    } else {
        digitalWrite(PIN_VIBRO_MOTOR, HIGH);
        delay(150);
        digitalWrite(PIN_VIBRO_MOTOR, LOW);
    }
}

void loop() {
    checkInputs();

    switch (g_state) {
        case STATE_IDLE:
            checkMotionEvents();
            if (millis() > DEEP_SLEEP_TIMEOUT_MS && !g_bleConnected && !g_bleActive) {
                enterDeepSleep();
            }
            break;

        case STATE_COUNTDOWN:
            handleStateCountdown();
            break;

        case STATE_DISPATCH:
            handleStateDispatch();
            break;

        case STATE_ALARM_ACTIVE:
            handleStateAlarmActive();
            break;
    }

    delay(20);
}

void initHardware() {
    pinMode(PIN_REED_SW, INPUT_PULLUP);
    pinMode(PIN_LIS3DH_INT, INPUT);
    pinMode(PIN_BT_SWITCH, INPUT_PULLUP);

    pinMode(PIN_VIBRO_MOTOR, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_SIM_PWRKEY, OUTPUT);

    digitalWrite(PIN_VIBRO_MOTOR, LOW);
    digitalWrite(PIN_STATUS_LED, LOW);
    digitalWrite(PIN_SIM_PWRKEY, HIGH);

    Wire.begin(PIN_SDA, PIN_SCL);
    SerialSIM.begin(SIM_UART_BAUD, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
}

void checkInputs() {
    // Bluetooth Switch (GPIO 0) Check
    static int lastBtSwitchState = -1;
    static unsigned long lastBtToggleTime = 0;
    int currentBtSwitchState = digitalRead(PIN_BT_SWITCH);

    if (currentBtSwitchState != lastBtSwitchState && (millis() - lastBtToggleTime > BT_DEBOUNCE_MS)) {
        lastBtToggleTime = millis();
        lastBtSwitchState = currentBtSwitchState;

        if (currentBtSwitchState == LOW) {
            if (!g_bleActive) {
                startBLE();
            }
        } else {
            if (g_bleActive) {
                stopBLE();
            }
        }
    }

    // Reed Switch (GPIO 2) Trigger Check
    static unsigned long lastReedTrigger = 0;
    int reedState = digitalRead(PIN_REED_SW);

#if REED_MODE_SWIPE
    if (reedState == LOW && (millis() - lastReedTrigger > 1000)) {
        lastReedTrigger = millis();
        if (g_state == STATE_COUNTDOWN) {
            cancelEmergency();
        } else if (g_state == STATE_IDLE) {
            triggerEmergency("Magnet SOS Swipe");
        }
    }
#else
    if (reedState == HIGH && (millis() - lastReedTrigger > 1000)) {
        lastReedTrigger = millis();
        if (g_state == STATE_IDLE) {
            triggerEmergency("Bracelet Clasp Removal");
        }
    }
#endif
}

void startBLE() {
    if (g_bleActive) return;

#if HAVE_BLE_STACK
  #if defined(USE_NIMBLE_LIB)
    NimBLEDevice::init(g_deviceSerial.c_str());
    NimBLEServer* pServer = NimBLEDevice::createServer();
    NimBLEService* pService = pServer->createService(BLE_SERVICE_UUID);
    pService->start();
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->start();
  #elif defined(USE_IDF_BLE)
    if (!g_bleInitDone) {
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_bt_controller_init(&bt_cfg);
        esp_bt_controller_enable(ESP_BT_MODE_BLE);
        esp_ble_gatts_register_callback(gattsEventHandler);
        esp_ble_gap_register_callback(gapEventHandler);
        esp_ble_gatts_app_register(0);
        esp_ble_gap_set_device_name(g_deviceSerial.c_str());
        esp_ble_gap_config_adv_data(&g_advData);
        g_bleInitDone = true;
    } else {
        esp_ble_gap_start_advertising(&g_advParams);
    }
  #endif
#endif

    g_bleActive = true;
    Serial.println("Bluetooth turned ON successfully!");

    // Feedback pulse: 2 short vibrations + 2 LED flashes
    for (int i = 0; i < 2; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        digitalWrite(PIN_VIBRO_MOTOR, HIGH);
        delay(100);
        digitalWrite(PIN_STATUS_LED, LOW);
        digitalWrite(PIN_VIBRO_MOTOR, LOW);
        delay(100);
    }
}

void stopBLE() {
    if (!g_bleActive) return;

#if HAVE_BLE_STACK
  #if defined(USE_NIMBLE_LIB)
    NimBLEDevice::deinit(true);
  #elif defined(USE_IDF_BLE)
    esp_ble_gap_stop_advertising();
  #endif
#endif

    g_bleActive = false;
    g_bleConnected = false;

    Serial.println("Bluetooth turned OFF successfully!");

    // Feedback pulse: 1 long vibration + 1 long LED flash
    digitalWrite(PIN_STATUS_LED, HIGH);
    digitalWrite(PIN_VIBRO_MOTOR, HIGH);
    delay(300);
    digitalWrite(PIN_STATUS_LED, LOW);
    digitalWrite(PIN_VIBRO_MOTOR, LOW);
}

void initLIS3DH() {
    Wire.beginTransmission(LIS3DH_ADDR_PRIMARY);
    if (Wire.endTransmission() == 0) {
        g_lis3dhAddr = LIS3DH_ADDR_PRIMARY;
        g_lis3dhFound = true;
    } else {
        Wire.beginTransmission(LIS3DH_ADDR_ALT);
        if (Wire.endTransmission() == 0) {
            g_lis3dhAddr = LIS3DH_ADDR_ALT;
            g_lis3dhFound = true;
        }
    }

    if (g_lis3dhFound) {
        Serial.print("LIS3DH Accelerometer detected at 0x");
        Serial.println(g_lis3dhAddr, HEX);

        Wire.beginTransmission(g_lis3dhAddr);
        Wire.write(0x20);
        Wire.write(0x57);
        Wire.endTransmission();

        Wire.beginTransmission(g_lis3dhAddr);
        Wire.write(0x23);
        Wire.write(0x20);
        Wire.endTransmission();

        Wire.beginTransmission(g_lis3dhAddr);
        Wire.write(0x22);
        Wire.write(0x40);
        Wire.endTransmission();

        Wire.beginTransmission(g_lis3dhAddr);
        Wire.write(0x32);
        Wire.write(0x20);
        Wire.endTransmission();

        Wire.beginTransmission(g_lis3dhAddr);
        Wire.write(0x33);
        Wire.write(0x02);
        Wire.endTransmission();

        Wire.beginTransmission(g_lis3dhAddr);
        Wire.write(0x30);
        Wire.write(0x2A);
        Wire.endTransmission();
    } else {
        Serial.println("WARNING: LIS3DH Accelerometer not detected on I2C bus!");
    }
}

void readLIS3DH(float &ax, float &ay, float &az) {
    if (!g_lis3dhFound) {
        ax = ay = az = 0.0f;
        return;
    }

    Wire.beginTransmission(g_lis3dhAddr);
    Wire.write(0x28 | 0x80);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(g_lis3dhAddr, (uint8_t)6) == 6) {
        int16_t rawX = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t rawY = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t rawZ = (int16_t)(Wire.read() | (Wire.read() << 8));

        ax = (rawX >> 4) * 0.004f;
        ay = (rawY >> 4) * 0.004f;
        az = (rawZ >> 4) * 0.004f;
    }
}

void checkMotionEvents() {
    if (millis() - g_lastMotionSample < 50) return;
    g_lastMotionSample = millis();

    float ax, ay, az;
    readLIS3DH(ax, ay, az);
    float totalG = sqrt(ax*ax + ay*ay + az*az);

    g_accelBuf[g_bufIdx] = totalG;
    g_bufIdx = (g_bufIdx + 1) % 16;

    if (totalG > IMPACT_G_THRESHOLD) {
        Serial.print("High impact detected! G=");
        Serial.println(totalG);
        delay(STILLNESS_TIME_MS);

        float stAx, stAy, stAz;
        readLIS3DH(stAx, stAy, stAz);
        float postG = sqrt(stAx*stAx + stAy*stAy + stAz*stAz);

        if (abs(postG - 1.0f) < 0.4f) {
            triggerEmergency("Fall Impact + Stillness Detected");
            return;
        }
    }

    float sum = 0.0f;
    for (int i = 0; i < 16; i++) sum += g_accelBuf[i];
    float mean = sum / 16.0f;

    float variance = 0.0f;
    for (int i = 0; i < 16; i++) {
        float diff = g_accelBuf[i] - mean;
        variance += diff * diff;
    }
    variance /= 16.0f;

    if (variance > STRUGGLE_VAR_THRES) {
        static unsigned long struggleStart = 0;
        if (struggleStart == 0) struggleStart = millis();
        else if (millis() - struggleStart > STRUGGLE_SUSTAIN_MS) {
            struggleStart = 0;
            triggerEmergency("Violent Struggle / Attack Detected");
        }
    } else {
        static unsigned long struggleStart = 0;
        struggleStart = 0;
    }
}

void triggerEmergency(String reason) {
    if (g_state != STATE_IDLE) return;

    Serial.print("--- EMERGENCY TRIGGERED: ");
    Serial.print(reason);
    Serial.println(" ---");

    g_state = STATE_COUNTDOWN;
    g_countdownStart = millis();
}

void cancelEmergency() {
    if (g_state != STATE_COUNTDOWN) return;

    Serial.println("Emergency alert CANCELLED by user.");
    g_state = STATE_IDLE;

    digitalWrite(PIN_VIBRO_MOTOR, LOW);
    digitalWrite(PIN_STATUS_LED, LOW);
}

void handleStateCountdown() {
    unsigned long elapsed = millis() - g_countdownStart;

    if (elapsed >= CANCEL_WINDOW_MS) {
        g_state = STATE_DISPATCH;
        digitalWrite(PIN_VIBRO_MOTOR, LOW);
        digitalWrite(PIN_STATUS_LED, LOW);
        return;
    }

    int phase = (elapsed / 250) % 2;
    digitalWrite(PIN_VIBRO_MOTOR, phase ? HIGH : LOW);
    digitalWrite(PIN_STATUS_LED, phase ? HIGH : LOW);
}

void handleStateDispatch() {
    Serial.println("20s window expired. Dispatching SOS SMS...");

    powerOnModem();

    float lat = 0.0f, lng = 0.0f;
    bool hasFix = fetchGNSSLocation(lat, lng);

    uint8_t bat = readBatteryLevel();
    String message = "EMERGENCY SOS! Raksha Band wearer needs help!";

    if (hasFix) {
        message += "\nLocation: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lng, 6);
    } else {
        message += "\nLocation: GNSS fixing... (Cell tower fallback active)";
    }
    message += "\nBattery: " + String(bat) + "%";

    Serial.println("Sending SMS payload:");
    Serial.println(message);

    bool smsSent = sendSMS(g_emergencyPhone, message);
    if (smsSent) {
        Serial.println("SOS SMS successfully sent!");
    } else {
        Serial.println("SMS sending failed! Retrying once...");
        delay(2000);
        sendSMS(g_emergencyPhone, message);
    }

    g_state = STATE_ALARM_ACTIVE;
}

void handleStateAlarmActive() {
    static unsigned long lastBeacon = 0;
    if (millis() - lastBeacon > 1500) {
        lastBeacon = millis();
        digitalWrite(PIN_STATUS_LED, HIGH);
        digitalWrite(PIN_VIBRO_MOTOR, HIGH);
        delay(100);
        digitalWrite(PIN_STATUS_LED, LOW);
        digitalWrite(PIN_VIBRO_MOTOR, LOW);
    }

    static unsigned long alarmStart = 0;
    if (alarmStart == 0) alarmStart = millis();
    else if (millis() - alarmStart > 300000UL) {
        alarmStart = 0;
        g_state = STATE_IDLE;
    }
}

bool powerOnModem() {
    Serial.println("Checking SIM7000E power state...");
    String resp = sendAT("AT", 1000);
    if (resp.indexOf("OK") != -1) {
        return true;
    }

    Serial.println("Pulsing SIM7000E PWRKEY...");
    digitalWrite(PIN_SIM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(PIN_SIM_PWRKEY, HIGH);
    delay(3000);

    resp = sendAT("AT", 2000);
    return (resp.indexOf("OK") != -1);
}

String sendAT(String cmd, unsigned long timeoutMs) {
    while (SerialSIM.available()) SerialSIM.read();

    SerialSIM.println(cmd);
    String response = "";
    unsigned long start = millis();

    while (millis() - start < timeoutMs) {
        while (SerialSIM.available()) {
            char c = SerialSIM.read();
            response += c;
        }
    }
    return response;
}

bool fetchGNSSLocation(float &lat, float &lng) {
    Serial.println("Powering GNSS engine...");
    sendAT("AT+CGNSPWR=1", 1000);
    delay(1000);

    unsigned long start = millis();
    while (millis() - start < 15000) {
        String resp = sendAT("AT+CGNSINF", 1000);
        int idx = resp.indexOf("+CGNSINF:");
        if (idx != -1) {
            String data = resp.substring(idx + 10);
            int p1 = data.indexOf(',');
            int p2 = data.indexOf(',', p1 + 1);
            int p3 = data.indexOf(',', p2 + 1);
            int p4 = data.indexOf(',', p3 + 1);
            int p5 = data.indexOf(',', p4 + 1);

            if (p1 != -1 && p2 != -1) {
                int fixStatus = data.substring(p1 + 1, p2).toInt();
                if (fixStatus == 1 && p3 != -1 && p4 != -1 && p5 != -1) {
                    lat = data.substring(p3 + 1, p4).toFloat();
                    lng = data.substring(p4 + 1, p5).toFloat();
                    return true;
                }
            }
        }
        delay(1000);
    }
    return false;
}

bool sendSMS(String number, String message) {
    sendAT("AT+CMGF=1", 1000);
    String cmd = "AT+CMGS=\"" + number + "\"";
    SerialSIM.println(cmd);
    delay(500);

    SerialSIM.print(message);
    SerialSIM.write(0x1A);

    String resp = sendAT("", 5000);
    return (resp.indexOf("OK") != -1 || resp.indexOf("+CMGS") != -1);
}

uint8_t readBatteryLevel() {
    uint32_t adcRaw = analogRead(PIN_BATTERY_ADC);
    float voltage = (adcRaw / 4095.0f) * 3.3f * 2.0f;
    if (voltage >= 4.2f) return 100;
    if (voltage <= 3.2f) return 0;
    return (uint8_t)((voltage - 3.2f) / 1.0f * 100.0f);
}

void enterDeepSleep() {
    Serial.println("Entering Ultra Low-Power Deep Sleep...");

    digitalWrite(PIN_VIBRO_MOTOR, LOW);
    digitalWrite(PIN_STATUS_LED, LOW);
    sendAT("AT+CPOWD=1", 1000);

    esp_deep_sleep_enable_gpio_wakeup((1ULL << PIN_REED_SW) | (1ULL << PIN_LIS3DH_INT), ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
}
