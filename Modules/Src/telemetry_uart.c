/**
 * ==============================================================================
 * @file    telemetry_uart.c
 * @brief   Telemetry Communication & Command Protocol Implementation.
 *          Target: STM32F401CCU6 (USART1 @ 115200bps)
 * ==============================================================================
 */

#include "telemetry_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private module state */
static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_byte_tmp = 0;
static uint8_t s_is_streaming = 0;
static uint8_t s_ble_connected = 0;

/* Circular Ring Buffer for non-blocking UART RX */
static uint8_t s_rx_ringbuf[TELEMETRY_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;

/* Line accumulation buffer for ASCII command parsing */
static char s_line_buf[TELEMETRY_CMD_LINE_SIZE];
static uint16_t s_line_idx = 0;

/* -----------------------------------------------------------------------------
 * Private Command Parser
 * -------------------------------------------------------------------------- */

static void ProcessCommandLine(char *cmd_line)
{
    // Trim leading spaces
    while (*cmd_line == ' ' || *cmd_line == '\t') {
        cmd_line++;
    }
    if (*cmd_line == '\0') {
        return;
    }

    // Filter unsolicited bootloader logs or debug output from ESP32-S3
    if (strncmp(cmd_line, "ESP-ROM:", 8) == 0 ||
        strncmp(cmd_line, "rst:", 4) == 0 ||
        strncmp(cmd_line, "boot:", 5) == 0 ||
        strncmp(cmd_line, "ets ", 4) == 0 ||
        strncmp(cmd_line, "load:", 5) == 0 ||
        strncmp(cmd_line, "entry ", 6) == 0 ||
        strncmp(cmd_line, "[I]", 3) == 0 ||
        strncmp(cmd_line, "[E]", 3) == 0 ||
        strncmp(cmd_line, "[W]", 3) == 0 ||
        strncmp(cmd_line, "[D]", 3) == 0) {
        return;
    }

    // 1. Handshake Queries (Both STM32 standard PING and ESP32-S3 bridge ping)
    if (strncmp(cmd_line, "ESP:PING", 8) == 0) {
        Telemetry_SendString("ESP:PONG\n");
        return;
    }
    if (strncmp(cmd_line, "PING", 4) == 0) {
        Telemetry_SendString("PONG\n");
        return;
    }

    // 2. ESP32-S3 BLE Connection Event Handlers (Fail-Safe Mechanism)
    if (strncmp(cmd_line, "BLE:CONNECTED", 13) == 0 || strcmp(cmd_line, "CONNECTED") == 0) {
        s_ble_connected = 1;
        Telemetry_SendString("OK_BLE:CONNECTED\n");
        return;
    }
    if (strncmp(cmd_line, "BLE:DISCONNECTED", 16) == 0 || strcmp(cmd_line, "DISCONNECTED") == 0) {
        s_ble_connected = 0;
        RobotFSM_SetCommand(CMD_STOP); // Immediately halt robot for safety
        Telemetry_SendString("OK_BLE:DISCONNECTED\n");
        return;
    }

    // 3. Status Query
    if (strcmp(cmd_line, "STATUS") == 0 || strcmp(cmd_line, "CMD:STATUS") == 0) {
        char status_buf[80];
        snprintf(status_buf, sizeof(status_buf), "STATUS:%s,BLE:%u\n",
                 RobotFSM_GetStateString(), (unsigned int)s_ble_connected);
        Telemetry_SendString(status_buf);
        return;
    }

    // 4. Telemetry Stream Control
    if (strncmp(cmd_line, "START_STREAM", 12) == 0) {
        s_is_streaming = 1;
        Telemetry_SendString("STREAM_STARTED\n");
        return;
    }
    if (strncmp(cmd_line, "STOP_STREAM", 11) == 0) {
        s_is_streaming = 0;
        Telemetry_SendString("STREAM_STOPPED\n");
        return;
    }

    // 5. Hot PID Tuning Command
    // Format: SET_PID:<ang_kp>,<ang_ki>,<ang_kd>,<spd_kp>,<spd_ki>
    if (strncmp(cmd_line, "SET_PID:", 8) == 0) {
        float ang_kp = 0.0f, ang_ki = 0.0f, ang_kd = 0.0f;
        float spd_kp = 0.0f, spd_ki = 0.0f;
        int parsed = sscanf(cmd_line + 8, "%f,%f,%f,%f,%f",
                            &ang_kp, &ang_ki, &ang_kd, &spd_kp, &spd_ki);
        if (parsed >= 5) {
            CascadePID_SetGains(ang_kp, ang_ki, ang_kd, spd_kp, spd_ki);
            char ack[64];
            snprintf(ack, sizeof(ack), "OK_PID:%.2f,%.3f,%.3f,%.2f,%.3f\n",
                     ang_kp, ang_ki, ang_kd, spd_kp, spd_ki);
            Telemetry_SendString(ack);
        } else {
            Telemetry_SendString("ERR:SET_PID_SYNTAX\n");
        }
        return;
    }

    // 6. Motion Control Commands (Full string and single-letter short forms)
    if (strcmp(cmd_line, "CMD:FORWARD") == 0 || strcmp(cmd_line, "FORWARD") == 0 || strcmp(cmd_line, "F") == 0) {
        RobotFSM_SetCommand(CMD_FORWARD);
        Telemetry_SendString("OK_CMD:FORWARD\n");
        return;
    }
    if (strcmp(cmd_line, "CMD:BACKWARD") == 0 || strcmp(cmd_line, "BACKWARD") == 0 || strcmp(cmd_line, "B") == 0) {
        RobotFSM_SetCommand(CMD_BACKWARD);
        Telemetry_SendString("OK_CMD:BACKWARD\n");
        return;
    }
    if (strcmp(cmd_line, "CMD:LEFT") == 0 || strcmp(cmd_line, "LEFT") == 0 || strcmp(cmd_line, "L") == 0) {
        RobotFSM_SetCommand(CMD_TURN_LEFT);
        Telemetry_SendString("OK_CMD:LEFT\n");
        return;
    }
    if (strcmp(cmd_line, "CMD:RIGHT") == 0 || strcmp(cmd_line, "RIGHT") == 0 || strcmp(cmd_line, "R") == 0) {
        RobotFSM_SetCommand(CMD_TURN_RIGHT);
        Telemetry_SendString("OK_CMD:RIGHT\n");
        return;
    }
    if (strcmp(cmd_line, "CMD:STOP") == 0 || strcmp(cmd_line, "STOP") == 0 || strcmp(cmd_line, "S") == 0) {
        RobotFSM_SetCommand(CMD_STOP);
        Telemetry_SendString("OK_CMD:STOP\n");
        return;
    }

    // Unknown command
    Telemetry_SendString("ERR:UNKNOWN_CMD\n");
}

/* -----------------------------------------------------------------------------
 * Public API Implementation
 * -------------------------------------------------------------------------- */

void Telemetry_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_head = 0;
    s_rx_tail = 0;
    s_line_idx = 0;
    s_is_streaming = 0;
    s_ble_connected = 0;

    Telemetry_StartReceiveIT();
}

void Telemetry_StartReceiveIT(void)
{
    if (s_huart != NULL) {
        HAL_UART_Receive_IT(s_huart, &s_rx_byte_tmp, 1);
    }
}

void Telemetry_OnRxByte(uint8_t byte)
{
    uint16_t next_head = (s_rx_head + 1) % TELEMETRY_RX_BUF_SIZE;
    if (next_head != s_rx_tail) {
        s_rx_ringbuf[s_rx_head] = byte;
        s_rx_head = next_head;
    }

    // Re-arm single-byte reception
    if (s_huart != NULL) {
        HAL_UART_Receive_IT(s_huart, &s_rx_byte_tmp, 1);
    }
}

void Telemetry_ProcessIncoming(void)
{
    while (s_rx_tail != s_rx_head) {
        uint8_t ch = s_rx_ringbuf[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % TELEMETRY_RX_BUF_SIZE;

        if (ch == '\n' || ch == '\r') {
            if (s_line_idx > 0) {
                s_line_buf[s_line_idx] = '\0';
                ProcessCommandLine(s_line_buf);
                s_line_idx = 0;
            }
        } else {
            if (s_line_idx < (TELEMETRY_CMD_LINE_SIZE - 1)) {
                s_line_buf[s_line_idx++] = (char)ch;
            } else {
                // Buffer overflow protection: flush line
                s_line_idx = 0;
            }
        }
    }
}

void Telemetry_SendData(float angle, float gyro, int16_t pwm,
                        int32_t enc_left, int32_t enc_right, uint8_t state)
{
    if (s_huart == NULL) return;

    char tx_buf[96];
    int len = snprintf(tx_buf, sizeof(tx_buf), "DATA:%.2f,%.2f,%d,%ld,%ld,%u\n",
                       angle, gyro, (int)pwm, (long)enc_left, (long)enc_right, (unsigned int)state);

    if (len > 0) {
        HAL_UART_Transmit(s_huart, (uint8_t*)tx_buf, (uint16_t)len, 10);
    }
}

void Telemetry_SendString(const char *str)
{
    if (s_huart == NULL || str == NULL) return;
    uint16_t len = (uint16_t)strlen(str);
    HAL_UART_Transmit(s_huart, (uint8_t*)str, len, 50);
}

uint8_t Telemetry_IsStreaming(void)
{
    return s_is_streaming;
}

void Telemetry_SetStreaming(uint8_t enable)
{
    s_is_streaming = enable ? 1 : 0;
}

uint8_t Telemetry_IsBLEConnected(void)
{
    return s_ble_connected;
}

void Telemetry_SendStatus(float angle, float speed, uint8_t state)
{
    if (s_huart == NULL) return;
    char buf[80];
    snprintf(buf, sizeof(buf), "STATUS:%.2f,%.2f,%u,BLE:%u\n",
             angle, speed, (unsigned int)state, (unsigned int)s_ble_connected);
    Telemetry_SendString(buf);
}

/* -----------------------------------------------------------------------------
 * HAL UART Callback Hook
 * -------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_huart != NULL && huart->Instance == s_huart->Instance) {
        Telemetry_OnRxByte(s_rx_byte_tmp);
    }
}
