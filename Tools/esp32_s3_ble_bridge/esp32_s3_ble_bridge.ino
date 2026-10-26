/**
 * ==============================================================================
 * Project: STM32F401CCU6 Self-Balancing Robot
 * File   : esp32_s3_ble_bridge.ino
 * Target : ESP32-S3 DevKitC-1 (used as external standalone BLE 5.0 module)
 * 
 * Description:
 * High-performance, transparent BLE-to-UART Bridge using Nordic UART Service (NUS).
 * Replaces legacy HC-05 classic Bluetooth module with modern Bluetooth 5.0 LE.
 * 
 * Hardware Connections:
 * - ESP32-S3 GPIO 4 (UART1 TX) ---> STM32F401 PA10 (USART1 RX)
 * - ESP32-S3 GPIO 5 (UART1 RX) <--- STM32F401 PA9  (USART1 TX)
 * - ESP32-S3 GND               <---> STM32F401 GND  (Common Ground)
 * - ESP32-S3 5V (or VIN)       <--- External 5V rail / Buck converter
 * 
 * Logic Level: Both STM32F401 and ESP32-S3 use 3.3V logic (directly compatible).
 * 
 * Features:
 * 1. Standard Nordic UART Service (NUS) profile:
 *    - Fully compatible with "Serial Bluetooth Terminal" (Android),
 *      "BLE Terminal" / "nRF Connect" (iOS/Android), and Web Bluetooth.
 * 2. Automated Fail-Safe Safety:
 *    - Sends "BLE:CONNECTED\n" upon phone connection.
 *    - Sends "BLE:DISCONNECTED\n" upon connection loss -> triggers STM32 auto-brake.
 * 3. Transparent Bidirectional Stream:
 *    - Phone App (BLE) ---> ESP32-S3 ---> STM32 UART1 (Commands: F, B, L, R, S, SET_PID)
 *    - STM32 UART1 ---> ESP32-S3 ---> Phone App (BLE) (Telemetry: DATA:angle,gyro,pwm...)
 * 4. Dual USB / Native CDC debug monitor for easy terminal inspection.
 * ==============================================================================
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* -----------------------------------------------------------------------------
 * Hardware Pinout Configuration
 * -------------------------------------------------------------------------- */
#define STM32_UART_BAUDRATE     115200
#define STM32_UART_NUM          1

// Hardware UART pins connected to STM32 USART1 (PA9/PA10)
#define PIN_TO_STM32_RX         4   // ESP32-S3 TX1 -> STM32 PA10 (USART1 RX)
#define PIN_FROM_STM32_TX       5   // ESP32-S3 RX1 <- STM32 PA9  (USART1 TX)

// Device Advertising Name
#define BLE_DEVICE_NAME         "STM32_Balance_Car"

// Nordic UART Service (NUS) Standard 128-bit UUIDs
#define SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Write from Phone
#define CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // Notify to Phone

/* -----------------------------------------------------------------------------
 * Global State
 * -------------------------------------------------------------------------- */
BLEServer          *pServer          = nullptr;
BLECharacteristic  *pTxCharacteristic = nullptr;
BLECharacteristic  *pRxCharacteristic = nullptr;

volatile bool deviceConnected    = false;
bool          oldDeviceConnected = false;

// Buffer for packing UART telemetry to BLE notifications
#define BLE_TX_BUF_SIZE 240
uint8_t  ble_tx_buffer[BLE_TX_BUF_SIZE];
uint16_t ble_tx_idx = 0;
unsigned long last_flush_time = 0;

/* -----------------------------------------------------------------------------
 * BLE Server Callbacks (Connection & Disconnection Events)
 * -------------------------------------------------------------------------- */
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServerInstance) override {
        deviceConnected = true;
        Serial.println("[BLE] Client connected!");
        // Notify STM32 of new connection
        Serial1.print("BLE:CONNECTED\n");
    }

    void onDisconnect(BLEServer* pServerInstance) override {
        deviceConnected = false;
        Serial.println("[BLE] Client disconnected! Informing STM32 for auto-stop...");
        // Fail-safe: Notify STM32 to immediately execute CMD_STOP
        Serial1.print("BLE:DISCONNECTED\n");
    }
};

/* -----------------------------------------------------------------------------
 * BLE Characteristic Callbacks (Incoming Commands from Phone/BLE Client)
 * -------------------------------------------------------------------------- */
class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        String rxValue = pChar->getValue();
        if (rxValue.length() > 0) {
            // Forward directly to STM32 UART1
            Serial1.write((const uint8_t*)rxValue.c_str(), rxValue.length());

            // Echo to USB Serial debug console
            Serial.print("[BLE->STM32] ");
            Serial.write((const uint8_t*)rxValue.c_str(), rxValue.length());
            if (!rxValue.endsWith("\n") && !rxValue.endsWith("\r")) {
                Serial.println();
            }
        }
    }
};

/* -----------------------------------------------------------------------------
 * Helper: Flush buffered UART data to BLE client via Notification
 * -------------------------------------------------------------------------- */
void flushBleTxBuffer() {
    if (ble_tx_idx == 0 || !deviceConnected || pTxCharacteristic == nullptr) {
        ble_tx_idx = 0;
        return;
    }

    pTxCharacteristic->setValue(ble_tx_buffer, ble_tx_idx);
    pTxCharacteristic->notify();
    ble_tx_idx = 0;
    last_flush_time = millis();
}

/* -----------------------------------------------------------------------------
 * Arduino Setup
 * -------------------------------------------------------------------------- */
void setup() {
    // 1. Initialize USB Serial for PC debug logs
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("==================================================");
    Serial.println(" ESP32-S3 Standalone BLE 5.0 UART Bridge for STM32");
    Serial.println(" Target: STM32F401CCU6 Self-Balancing Robot");
    Serial.println("==================================================");

    // 2. Initialize Hardware Serial1 connected to STM32F401 (USART1 PA9/PA10)
    Serial1.begin(STM32_UART_BAUDRATE, SERIAL_8N1, PIN_FROM_STM32_TX, PIN_TO_STM32_RX);
    Serial.printf("[INIT] Serial1 initialized: TX=GPIO%d, RX=GPIO%d @ %d baud\n",
                  PIN_TO_STM32_RX, PIN_FROM_STM32_TX, STM32_UART_BAUDRATE);

    // 3. Initialize BLE Subsystem
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(256); // Request larger MTU for high-rate telemetry streaming

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Create Nordic UART Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // TX Characteristic (ESP32 -> Phone, Notify)
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    // RX Characteristic (Phone -> ESP32, Write / Write without response)
    pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    // Start Service
    pService->start();

    // 4. Start Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Functions that help with iPhone connections
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising started. Waiting for smartphone connection...");
}

/* -----------------------------------------------------------------------------
 * Arduino Main Loop
 * -------------------------------------------------------------------------- */
void loop() {
    // Forward incoming data from STM32 UART1 -> Phone (BLE Notify)
    while (Serial1.available()) {
        uint8_t c = (uint8_t)Serial1.read();

        // Echo telemetry to PC debug console if desired
        // Serial.write(c);

        if (deviceConnected) {
            ble_tx_buffer[ble_tx_idx++] = c;

            // Transmit chunk if buffer is almost full or newline delimiter reached
            if (ble_tx_idx >= (BLE_TX_BUF_SIZE - 4) || c == '\n') {
                flushBleTxBuffer();
            }
        }
    }

    // Flush periodic timeout (every 20ms) if partial packet remains in buffer
    if (ble_tx_idx > 0 && (millis() - last_flush_time > 20)) {
        flushBleTxBuffer();
    }

    // Handle advertising restart after disconnection
    if (!deviceConnected && oldDeviceConnected) {
        delay(300); // Allow BLE stack to finalize
        pServer->startAdvertising();
        Serial.println("[BLE] Restarted advertising after disconnect.");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // Forward any typing from USB Serial console to STM32 for testing
    while (Serial.available()) {
        uint8_t usb_char = Serial.read();
        Serial1.write(usb_char);
    }
}
