/**
 * ==============================================================================
 * Project: STM32F401CCU6 + ESP32-S3 Hardware Pin Diagnostic & Crosstalk Scanner
 * File   : esp32_pin_scanner.ino
 * Target : ESP32-S3 DevKitC-1 (N16R8)
 * ==============================================================================
 */

#include <Arduino.h>

const uint8_t CANDIDATES[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21,
    38, 39, 40, 41, 42, 43, 44, 45, 47, 48
};
const uint8_t NUM_CANDIDATES = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

void setupAllInputPullup() {
    for (uint8_t i = 0; i < NUM_CANDIDATES; i++) {
        pinMode(CANDIDATES[i], INPUT_PULLUP);
    }
}

void printStatus() {
    Serial.print("PIN_STATUS:");
    for (uint8_t i = 0; i < NUM_CANDIDATES; i++) {
        uint8_t p = CANDIDATES[i];
        Serial.printf(" %d=%d", p, digitalRead(p));
    }
    Serial.println();
}

void checkShorts() {
    Serial.println("--- SHORT CIRCUIT SCAN START ---");
    uint8_t count = 0;
    setupAllInputPullup();
    delay(10);

    for (uint8_t i = 0; i < NUM_CANDIDATES; i++) {
        uint8_t test_pin = CANDIDATES[i];

        pinMode(test_pin, OUTPUT);
        digitalWrite(test_pin, LOW);
        delayMicroseconds(200);

        for (uint8_t j = 0; j < NUM_CANDIDATES; j++) {
            uint8_t other_pin = CANDIDATES[j];
            if (other_pin == test_pin) continue;
            if (digitalRead(other_pin) == LOW) {
                Serial.printf("SHORT: %d <-> %d\n", test_pin, other_pin);
                count++;
            }
        }

        pinMode(test_pin, INPUT_PULLUP);
        delayMicroseconds(100);
    }

    if (count == 0) {
        Serial.println("SHORT_SCAN_PASS: No short circuits detected");
    } else {
        Serial.printf("SHORT_SCAN_FAIL: %d shorts detected\n", count / 2);
    }
    Serial.println("--- SHORT CIRCUIT SCAN END ---");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    setupAllInputPullup();
    Serial.println("ESP32_DIAG_READY");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "STATUS") {
            printStatus();
        } else if (cmd == "TEST_SHORTS") {
            checkShorts();
        } else if (cmd.startsWith("PULL_LOW ")) {
            int pin = cmd.substring(9).toInt();
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
            Serial.printf("OK_PULL_LOW: %d\n", pin);
        } else if (cmd.startsWith("PULL_HIGH ")) {
            int pin = cmd.substring(10).toInt();
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
            Serial.printf("OK_PULL_HIGH: %d\n", pin);
        } else if (cmd.startsWith("RELEASE ")) {
            int pin = cmd.substring(8).toInt();
            pinMode(pin, INPUT_PULLUP);
            Serial.printf("OK_RELEASE: %d\n", pin);
        } else if (cmd.startsWith("PULSE ")) {
            int space_idx = cmd.indexOf(' ', 6);
            if (space_idx > 0) {
                int pin = cmd.substring(6, space_idx).toInt();
                int ms = cmd.substring(space_idx + 1).toInt();
                pinMode(pin, OUTPUT);
                digitalWrite(pin, LOW);
                delay(ms);
                digitalWrite(pin, HIGH);
                pinMode(pin, INPUT_PULLUP);
                Serial.printf("OK_PULSE: %d %dms\n", pin, ms);
            }
        } else if (cmd.startsWith("UART_LISTEN ")) {
            // UART_LISTEN <pin> <baud> <timeout_ms>
            int p1 = cmd.indexOf(' ', 12);
            int rx_pin = cmd.substring(12, p1).toInt();
            int p2 = cmd.indexOf(' ', p1 + 1);
            long baud = (p2 > 0) ? cmd.substring(p1 + 1, p2).toInt() : 115200;
            long timeout = (p2 > 0) ? cmd.substring(p2 + 1).toInt() : 1000;

            Serial1.begin(baud, SERIAL_8N1, rx_pin, -1);
            delay(10);
            unsigned long t0 = millis();
            String received = "";
            while (millis() - t0 < (unsigned long)timeout) {
                while (Serial1.available()) {
                    char c = (char)Serial1.read();
                    received += c;
                }
            }
            Serial1.end();
            pinMode(rx_pin, INPUT_PULLUP);
            Serial.printf("UART_RX_RESULT: pin=%d bytes=%d data=\"%s\"\n",
                          rx_pin, received.length(), received.c_str());
        } else if (cmd.startsWith("UART_SEND ")) {
            // UART_SEND <tx_pin> <baud> <string>
            int p1 = cmd.indexOf(' ', 10);
            int tx_pin = cmd.substring(10, p1).toInt();
            int p2 = cmd.indexOf(' ', p1 + 1);
            long baud = cmd.substring(p1 + 1, p2).toInt();
            String payload = cmd.substring(p2 + 1);
            payload.replace("\\n", "\n");
            payload.replace("\\r", "\r");

            Serial1.begin(baud, SERIAL_8N1, -1, tx_pin);
            delay(10);
            Serial1.print(payload);
            Serial1.flush();
            delay(10);
            Serial1.end();
            pinMode(tx_pin, INPUT_PULLUP);
            Serial.printf("UART_TX_RESULT: pin=%d sent=%d\n", tx_pin, payload.length());
        } else {
            Serial.printf("UNKNOWN_CMD: %s\n", cmd.c_str());
        }
    }
}
