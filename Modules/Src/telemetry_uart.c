/**
 * ==============================================================================
 * @file    telemetry_uart.c
 * @brief   Telemetry Communication & Command Protocol Implementation.
 *          Target: STM32F401CCU6 (USART1 @ 115200bps)
 * ==============================================================================
 */

#include "telemetry_uart.h"
#include "mpu6050_filter.h"
#include "motor_encoder.h"
#include "motor_test.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private module state */
static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_byte_tmp = 0;
static uint8_t s_is_streaming = 0;
static uint8_t s_ble_connected = 0;
static void (*s_rx_hook)(uint8_t) = NULL;

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

    // 3. Status Query (Detailed hardware & telemetry status)
    if (strcmp(cmd_line, "STATUS") == 0 || strcmp(cmd_line, "CMD:STATUS") == 0) {
        char status_buf[140];
        const motor_encoder_data_t *enc = MotorEncoder_GetData();
        snprintf(status_buf, sizeof(status_buf),
                 "STATUS:STATE=%s,PITCH=%.2f,GYRO=%.2f,SPD=%.2f,ENC_L=%ld,ENC_R=%ld,MPU=%u,BLE=%u\n",
                 RobotFSM_GetStateString(),
                 MPU6050_GetPitch(),
                 MPU6050_GetGyroRate(),
                 enc->speed_avg_mps,
                 (long)enc->total_pulses_left,
                 (long)enc->total_pulses_right,
                 (unsigned int)MPU6050_IsConnected(),
                 (unsigned int)s_ble_connected);
        Telemetry_SendString(status_buf);
        return;
    }

    // 4. Set Balance Zero Pitch Offset
    // Format: CALIB_ZERO (auto capture current angle) or CALIB_ZERO:<angle>
    if (strncmp(cmd_line, "CALIB_ZERO", 10) == 0) {
        float offset = 0.0f;
        if (cmd_line[10] == ':') {
            offset = strtof(cmd_line + 11, NULL);
        } else {
            const mpu6050_data_t *imu = MPU6050_GetData();
            offset = imu->pitch + imu->accel_pitch_offset;
        }
        MPU6050_SetZeroOffset(offset);
        char ack[64];
        snprintf(ack, sizeof(ack), "OK_CALIB_ZERO:offset=%.2f\n", offset);
        Telemetry_SendString(ack);
        return;
    }

    // 5. Motor & Encoder Direction Polarities
    // Format: SET_POLARITY:<motor_l>,<motor_r>,<enc_l>,<enc_r>
    if (strncmp(cmd_line, "SET_POLARITY:", 13) == 0) {
        int ml = 1, mr = -1, el = 1, er = -1;
        if (sscanf(cmd_line + 13, "%d,%d,%d,%d", &ml, &mr, &el, &er) >= 4) {
            Motor_SetPolarity((int8_t)ml, (int8_t)mr, (int8_t)el, (int8_t)er);
            char ack[64];
            snprintf(ack, sizeof(ack), "OK_POLARITY:%d,%d,%d,%d\n", ml, mr, el, er);
            Telemetry_SendString(ack);
        } else {
            Telemetry_SendString("ERR:SET_POLARITY_SYNTAX\n");
        }
        return;
    }

    // 6. Hardware Motor Diagnostic Tests
    if (strcmp(cmd_line, "TEST:MOTOR_L") == 0) {
        Telemetry_SendString("TEST:MOTOR_L_RUN (FWD)\n");
        Motor_TestRun(1500, 0);
        HAL_Delay(600);
        Motor_Disable();
        Telemetry_SendString("TEST:MOTOR_L_STOP\n");
        return;
    }
    if (strcmp(cmd_line, "TEST:MOTOR_L_REV") == 0) {
        Telemetry_SendString("TEST:MOTOR_L_RUN (REV)\n");
        Motor_TestRun(-1500, 0);
        HAL_Delay(600);
        Motor_Disable();
        Telemetry_SendString("TEST:MOTOR_L_STOP\n");
        return;
    }
    if (strcmp(cmd_line, "TEST:MOTOR_R") == 0) {
        Telemetry_SendString("TEST:MOTOR_R_RUN (FWD)\n");
        Motor_TestRun(0, 1500);
        HAL_Delay(600);
        Motor_Disable();
        Telemetry_SendString("TEST:MOTOR_R_STOP\n");
        return;
    }
    if (strcmp(cmd_line, "TEST:MOTOR_R_REV") == 0) {
        Telemetry_SendString("TEST:MOTOR_R_RUN (REV)\n");
        Motor_TestRun(0, -1500);
        HAL_Delay(600);
        Motor_Disable();
        Telemetry_SendString("TEST:MOTOR_R_STOP\n");
        return;
    }
    if (strcmp(cmd_line, "TEST:MOTORS") == 0) {
        Telemetry_SendString("TEST:MOTORS_RUN (FWD)\n");
        Motor_TestRun(1500, 1500);
        HAL_Delay(800);
        Motor_Disable();
        Telemetry_SendString("TEST:MOTORS_STOP\n");
        return;
    }
    if (strcmp(cmd_line, "TEST:MOTORS_REV") == 0) {
        Telemetry_SendString("TEST:MOTORS_RUN (REV)\n");
        Motor_TestRun(-1500, -1500);
        HAL_Delay(800);
        Motor_Disable();
        Telemetry_SendString("TEST:MOTORS_STOP\n");
        return;
    }
    if (strncmp(cmd_line, "MOTOR:", 6) == 0) {
        int pl = 0, pr = 0;
        if (sscanf(cmd_line + 6, "%d,%d", &pl, &pr) == 2) {
            Motor_TestRun((int16_t)pl, (int16_t)pr);
            char resp[48];
            snprintf(resp, sizeof(resp), "OK_MOTOR:PWM_L=%d,PWM_R=%d\n", pl, pr);
            Telemetry_SendString(resp);
            return;
        }
    }
    if (strcmp(cmd_line, "TEST:AUTO") == 0) {
        Motor_RunAutoTestLoop();
        return;
    }
    if (strcmp(cmd_line, "TEST:STOP") == 0) {
        Motor_Disable();
        Telemetry_SendString("TEST:STOPPED\n");
        return;
    }

    // 7. Telemetry Stream Control
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
    Telemetry_SendString("\n[SYS] STM32F401 Self-Balancing Robot Initialized.\n");
}

void Telemetry_StartReceiveIT(void)
{
    if (s_huart != NULL) {
        HAL_UART_Receive_IT(s_huart, &s_rx_byte_tmp, 1);
    }
}

void Telemetry_RegisterRxHook(void (*hook)(uint8_t))
{
    s_rx_hook = hook;
}

void Telemetry_OnRxByte(uint8_t byte)
{
    if (s_rx_hook != NULL) {
        s_rx_hook(byte);
        if (s_huart != NULL) {
            HAL_UART_Receive_IT(s_huart, &s_rx_byte_tmp, 1);
        }
        return;
    }

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
    // Auto-revive UART RX if it got stalled due to line noise or overrun error
    if (s_huart != NULL && s_huart->RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(s_huart, &s_rx_byte_tmp, 1);
    }

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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_huart != NULL && huart->Instance == s_huart->Instance) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        HAL_UART_Receive_IT(s_huart, &s_rx_byte_tmp, 1);
    }
}
