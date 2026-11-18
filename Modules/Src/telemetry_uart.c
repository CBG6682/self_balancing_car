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
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>

/* Global state for LED manual override */
volatile uint8_t g_led_manual_mode = 0;

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
static volatile uint32_t s_last_rx_tick = 0;

/* Line accumulation buffer for ASCII command parsing */
static char s_line_buf[TELEMETRY_CMD_LINE_SIZE];
static uint16_t s_line_idx = 0;

/* Lightweight embedded float formatter to avoid newlib-nano printf_float heap/stack overhead */
static void format_float(char *out, size_t out_sz, float val, int decimals)
{
    long factor = 1;
    for (int i = 0; i < decimals; i++) factor *= 10;
    long total = (long)(val * factor);
    const char *sign = (val < 0.0f && total != 0) ? "-" : "";
    long abs_total = labs(total);
    long int_part = abs_total / factor;
    long frac_part = abs_total % factor;
    snprintf(out, out_sz, "%s%ld.%0*lu", sign, int_part, decimals, (unsigned long)frac_part);
}

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

    // 3. Status & Parameter Query (Comprehensive car telemetry and controller settings)
    if (strcasecmp(cmd_line, "PARAMS") == 0 || strcasecmp(cmd_line, "GET_PARAMS") == 0 ||
        strcasecmp(cmd_line, "INFO") == 0 || strcasecmp(cmd_line, "STATUS") == 0 ||
        strcasecmp(cmd_line, "CMD:STATUS") == 0) {
        const cascade_pid_t *pid = CascadePID_GetData();
        const motor_encoder_data_t *enc = MotorEncoder_GetData();
        char s_pitch[16], s_gyro[16], s_spd[16], s_off[16];
        char s_kp[16], s_ki[16], s_kd[16], s_skp[16], s_ski[16];
        format_float(s_pitch, sizeof(s_pitch), MPU6050_GetPitch(), 2);
        format_float(s_gyro, sizeof(s_gyro), MPU6050_GetGyroRate(), 2);
        format_float(s_spd, sizeof(s_spd), enc->speed_avg_mps, 2);
        format_float(s_off, sizeof(s_off), CascadePID_GetBalanceOffset(), 2);
        format_float(s_kp, sizeof(s_kp), pid->angle_kp, 2);
        format_float(s_ki, sizeof(s_ki), pid->angle_ki, 3);
        format_float(s_kd, sizeof(s_kd), pid->angle_kd, 3);
        format_float(s_skp, sizeof(s_skp), pid->speed_kp, 2);
        format_float(s_ski, sizeof(s_ski), pid->speed_ki, 3);

        char buf[512];
        snprintf(buf, sizeof(buf),
                 "\r\n=== THONG SO XE HIEN TAI ===\r\n"
                 " • PID Goc   : Kp=%s, Ki=%s, Kd=%s\r\n"
                 " • PI Toc do : sKp=%s, sKi=%s\r\n"
                 " • Offset    : %s deg\r\n"
                 " • Deadzone  : %u (ARR=999)\r\n"
                 " • Che do    : %s\r\n"
                 " • FSM State : %s\r\n"
                 " • Sensor    : Pitch=%s deg, Gyro=%s dps\r\n"
                 " • Van toc   : %s m/s (L=%ld, R=%ld)\r\n"
                 "============================\r\n",
                 s_kp, s_ki, s_kd,
                 s_skp, s_ski,
                 s_off,
                 (unsigned int)Motor_GetDeadzone(),
                 (g_system_mode == SYS_MODE_AUTO) ? "AUTO (TU CAN BANG)" : "OFF (DONG CO NGAT)",
                 RobotFSM_GetStateString(),
                 s_pitch,
                 s_gyro,
                 s_spd,
                 (long)enc->total_pulses_left,
                 (long)enc->total_pulses_right);
        Telemetry_SendString(buf);
        return;
    }

    // 4. Set / Get Balance Zero Pitch Offset
    // Format: SET_OFFSET:<angle>, OFFSET:<angle>, or CALIB_ZERO:<angle>
    if (strncasecmp(cmd_line, "SET_OFFSET:", 11) == 0 ||
        strncasecmp(cmd_line, "SET_OFFSET", 10) == 0 ||
        strncasecmp(cmd_line, "OFFSET:", 7) == 0 ||
        strncasecmp(cmd_line, "CALIB_ZERO", 10) == 0) {
        float offset = 0.0f;
        const char *p = strchr(cmd_line, ':');
        if (p) {
            offset = strtof(p + 1, NULL);
        } else {
            const mpu6050_data_t *imu = MPU6050_GetData();
            offset = imu->pitch + imu->accel_pitch_offset;
        }
        CascadePID_SetBalanceOffset(offset);
        char ack[64], s_off[16];
        format_float(s_off, sizeof(s_off), offset, 2);
        snprintf(ack, sizeof(ack), "OK_OFFSET:%s\r\n", s_off);
        Telemetry_SendString(ack);
        return;
    }
    if (strcasecmp(cmd_line, "GET_OFFSET") == 0 || strcasecmp(cmd_line, "OFFSET?") == 0) {
        char ack[64], s_off[16];
        format_float(s_off, sizeof(s_off), CascadePID_GetBalanceOffset(), 2);
        snprintf(ack, sizeof(ack), "OFFSET:%s\r\n", s_off);
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

    // 5. Hot PID Tuning Commands
    // Format: SET_PID:<ang_kp>,<ang_ki>,<ang_kd>,<spd_kp>,<spd_ki> or PID:<ang_kp>,...
    if (strncasecmp(cmd_line, "SET_PID:", 8) == 0 || strncasecmp(cmd_line, "PID:", 4) == 0) {
        char *p = strchr(cmd_line, ':');
        if (p) {
            char *next = p + 1;
            float vals[5];
            int count = 0;
            for (int i = 0; i < 5; i++) {
                while (*next == ' ' || *next == '\t') next++;
                if (*next == '\0') break;
                char *endptr = NULL;
                vals[i] = strtof(next, &endptr);
                if (endptr == next) break;
                count++;
                next = endptr;
                while (*next == ' ' || *next == '\t') next++;
                if (*next == ',') next++;
            }
            if (count >= 5) {
                CascadePID_SetGains(vals[0], vals[1], vals[2], vals[3], vals[4]);
                char s0[16], s1[16], s2[16], s3[16], s4[16];
                format_float(s0, sizeof(s0), vals[0], 2);
                format_float(s1, sizeof(s1), vals[1], 3);
                format_float(s2, sizeof(s2), vals[2], 3);
                format_float(s3, sizeof(s3), vals[3], 2);
                format_float(s4, sizeof(s4), vals[4], 3);
                char ack[96];
                snprintf(ack, sizeof(ack), "OK_PID:%s,%s,%s,%s,%s\r\n", s0, s1, s2, s3, s4);
                Telemetry_SendString(ack);
                return;
            }
        }
        Telemetry_SendString("ERR:SET_PID_SYNTAX (SET_PID:kp,ki,kd,skp,ski)\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "GET_PID") == 0 || strcasecmp(cmd_line, "PID?") == 0 || strcasecmp(cmd_line, "PID") == 0) {
        const cascade_pid_t *pid = CascadePID_GetData();
        char s0[16], s1[16], s2[16], s3[16], s4[16];
        format_float(s0, sizeof(s0), pid->angle_kp, 2);
        format_float(s1, sizeof(s1), pid->angle_ki, 3);
        format_float(s2, sizeof(s2), pid->angle_kd, 3);
        format_float(s3, sizeof(s3), pid->speed_kp, 2);
        format_float(s4, sizeof(s4), pid->speed_ki, 3);
        char ack[96];
        snprintf(ack, sizeof(ack), "PID:%s,%s,%s,%s,%s\r\n", s0, s1, s2, s3, s4);
        Telemetry_SendString(ack);
        return;
    }

    // 6. Operating Mode Commands (AUTO balancing vs STOP / OFF shutdown)
    if (strcasecmp(cmd_line, "AUTO") == 0 ||
        strcasecmp(cmd_line, "MODE:AUTO") == 0 ||
        strcasecmp(cmd_line, "START") == 0 ||
        strcasecmp(cmd_line, "RUN") == 0) {
        Motor_EnterMode_AUTO();
        Telemetry_SendString("OK_MODE:AUTO (Tu can bang BAT - Dung xe +/-10 deg de chay)\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "STOP") == 0 ||
        strcasecmp(cmd_line, "OFF") == 0 ||
        strcasecmp(cmd_line, "MODE:STOP") == 0 ||
        strcasecmp(cmd_line, "MODE:OFF") == 0 ||
        strcasecmp(cmd_line, "CMD:STOP") == 0) {
        Motor_EnterMode_OFF();
        RobotFSM_SetCommand(CMD_STOP);
        Telemetry_SendString("OK_MODE:STOP (Dong co da NGAT hoan toan)\r\n");
        return;
    }

    // 6b. Hardware Driver Pin Probing Commands (STBY locked strictly to 0.0V, safe multimeter testing)
    if (strcasecmp(cmd_line, "PROBE") == 0 ||
        strcasecmp(cmd_line, "PROBE:ALL") == 0 ||
        strcasecmp(cmd_line, "TEST_PINS") == 0 ||
        strcasecmp(cmd_line, "CHECK_PINS") == 0 ||
        strcasecmp(cmd_line, "PROBE_DRIVER") == 0) {
        Motor_EnterMode_BIN_Probe();
        Telemetry_SendString(
            "\r\n=== CHE DO KIEM TRA CHAN DRIVER (STBY KHOA 0V) ===\r\n"
            " • STBY (PA2) : 0.0V (KHOA AN TOAN, DONG CO KHONG CHAY)\r\n"
            " • AIN1 (PA4) : 3.3V (HIGH)\r\n"
            " • AIN2 (PA5) : 3.3V (HIGH)\r\n"
            " • BIN1 (PA3) : 3.3V (HIGH)\r\n"
            " • BIN2 (PA7) : 3.3V (HIGH)\r\n"
            " • PWMA (PB8) : 3.3V (PWM 100% DC)\r\n"
            " • PWMB (PB9) : 3.3V (PWM 100% DC)\r\n"
            "----------------------------------------------------\r\n"
            "-> Lay dong ho VOM do dien ap tung chan tren TB6612.\r\n"
            "-> Go 'STOP' hoac 'OFF' de ngat ve 0V.\r\n"
            "-> Go 'AUTO' de quay lai tu can bang.\r\n"
            "====================================================\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "PROBE:A") == 0) {
        Motor_ProbeDriverPins(1, 0, 1);
        Telemetry_SendString("OK_PROBE:A (AIN1=3.3V, AIN2=3.3V, PWMA=3.3V, BIN=0V, STBY=0V)\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "PROBE:B") == 0) {
        Motor_ProbeDriverPins(0, 1, 1);
        Telemetry_SendString("OK_PROBE:B (BIN1=3.3V, BIN2=3.3V, PWMB=3.3V, AIN=0V, STBY=0V)\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "PROBE:PWM") == 0) {
        Motor_ProbeDriverPins(0, 0, 1);
        Telemetry_SendString("OK_PROBE:PWM (PWMA=3.3V, PWMB=3.3V, AIN=0V, BIN=0V, STBY=0V)\r\n");
        return;
    }

    // 7. Motor Deadzone Tuning Commands
    if (strncasecmp(cmd_line, "SET_DEADZONE:", 13) == 0 ||
        strncasecmp(cmd_line, "DEADZONE:", 9) == 0) {
        const char *p = strchr(cmd_line, ':');
        int dz = atoi(p + 1);
        if (dz >= 0 && dz <= 400) {
            Motor_SetDeadzone((uint16_t)dz);
            char ack[48];
            snprintf(ack, sizeof(ack), "OK_DEADZONE:%d\r\n", dz);
            Telemetry_SendString(ack);
            return;
        }
        Telemetry_SendString("ERR:DEADZONE_RANGE (0..400)\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "GET_DEADZONE") == 0 || strcasecmp(cmd_line, "DEADZONE?") == 0) {
        char ack[48];
        snprintf(ack, sizeof(ack), "DEADZONE:%u\r\n", (unsigned int)Motor_GetDeadzone());
        Telemetry_SendString(ack);
        return;
    }

    // 8. Odometry / Encoder Reset
    if (strcasecmp(cmd_line, "RESET_ENC") == 0 || strcasecmp(cmd_line, "ZERO_ENC") == 0 || strcasecmp(cmd_line, "CLEAR_ENC") == 0) {
        MotorEncoder_ResetCounters();
        Telemetry_SendString("OK_ENC:RESET (0, 0)\r\n");
        return;
    }

    // 9. Built-in Help Menu
    if (strcasecmp(cmd_line, "HELP") == 0 || strcmp(cmd_line, "?") == 0) {
        Telemetry_SendString(
            "\r\n=== BANG LENH DIEU KHIEN BLUETOOTH ===\r\n"
            " • PARAMS / INFO : Xem toan bo thong so xe\r\n"
            " • GET_PID       : Xem 5 thong so PID\r\n"
            " • SET_PID:kp,ki,kd,skp,ski : Cai dat PID (VD: SET_PID:35,0.15,0.32,5,0.6)\r\n"
            " • OFFSET:val    : Chinh goc can bang (VD: OFFSET:0.00 hoac OFFSET:-0.20)\r\n"
            " • GET_OFFSET    : Xem goc can bang hien tai\r\n"
            " • AUTO / START  : Bat che do tu can bang\r\n"
            " • STOP / OFF    : Ngat dong co khan cap\r\n"
            " • PROBE         : Kich AIN, BIN, PWM len 3.3V (STBY khoa 0V do VOM)\r\n"
            " • DEADZONE:val  : Chinh vung chet (VD: DEADZONE:18)\r\n"
            " • RESET_ENC     : Reset quang duong encoder ve 0\r\n"
            " • F / B / L / R : Tien / Lui / Trai / Phai\r\n"
            " • S             : Dung di chuyen (giu thang bang tai cho)\r\n"
            "=======================================\r\n");
        return;
    }

    // 10. Onboard / Module LED Control Commands (Visual Verification without motors)
    if (strcasecmp(cmd_line, "CMD:LED_ON") == 0 ||
        strcasecmp(cmd_line, "LED:ON") == 0 ||
        strcasecmp(cmd_line, "LED_ON") == 0) {
        g_led_manual_mode = 1;
        Board_LED_Set(1); // 1 = LED ON (PC13 LOW)
        Telemetry_SendString("OK_LED:ON\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "CMD:LED_OFF") == 0 ||
        strcasecmp(cmd_line, "LED:OFF") == 0 ||
        strcasecmp(cmd_line, "LED_OFF") == 0) {
        g_led_manual_mode = 1;
        Board_LED_Set(0); // 0 = LED OFF (PC13 HIGH)
        Telemetry_SendString("OK_LED:OFF\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "CMD:LED_TOGGLE") == 0 ||
        strcasecmp(cmd_line, "LED:TOGGLE") == 0 ||
        strcasecmp(cmd_line, "LED_TOGGLE") == 0) {
        g_led_manual_mode = 1;
        Board_LED_Toggle();
        Telemetry_SendString("OK_LED:TOGGLE\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "CMD:LED_AUTO") == 0 ||
        strcasecmp(cmd_line, "LED:AUTO") == 0 ||
        strcasecmp(cmd_line, "LED_AUTO") == 0) {
        g_led_manual_mode = 0;
        Telemetry_SendString("OK_LED:AUTO\r\n");
        return;
    }

    // 11. Motion Control Commands (Full string and single-letter short forms)
    if (strcasecmp(cmd_line, "CMD:FORWARD") == 0 || strcasecmp(cmd_line, "FORWARD") == 0 || strcasecmp(cmd_line, "F") == 0) {
        RobotFSM_SetCommand(CMD_FORWARD);
        Board_LED_Toggle();
        Telemetry_SendString("OK_CMD:FORWARD\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "CMD:BACKWARD") == 0 || strcasecmp(cmd_line, "BACKWARD") == 0 || strcasecmp(cmd_line, "B") == 0) {
        RobotFSM_SetCommand(CMD_BACKWARD);
        Board_LED_Toggle();
        Telemetry_SendString("OK_CMD:BACKWARD\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "CMD:LEFT") == 0 || strcasecmp(cmd_line, "LEFT") == 0 || strcasecmp(cmd_line, "L") == 0) {
        RobotFSM_SetCommand(CMD_TURN_LEFT);
        Board_LED_Toggle();
        Telemetry_SendString("OK_CMD:LEFT\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "CMD:RIGHT") == 0 || strcasecmp(cmd_line, "RIGHT") == 0 || strcasecmp(cmd_line, "R") == 0) {
        RobotFSM_SetCommand(CMD_TURN_RIGHT);
        Board_LED_Toggle();
        Telemetry_SendString("OK_CMD:RIGHT\r\n");
        return;
    }
    if (strcasecmp(cmd_line, "S") == 0 || strcasecmp(cmd_line, "HALT") == 0) {
        RobotFSM_SetCommand(CMD_STOP);
        Board_LED_Toggle();
        Telemetry_SendString("OK_CMD:STOP\r\n");
        return;
    }

    // Unknown command
    Telemetry_SendString("ERR:UNKNOWN_CMD (Go 'HELP' de xem danh sach lenh)\r\n");
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
    s_last_rx_tick = HAL_GetTick();

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

    // 1. Immediate execution for known 1-character BLE/UART commands without trailing newline (after 20ms idle)
    if (s_line_idx == 1 && s_rx_tail == s_rx_head) {
        if ((HAL_GetTick() - s_last_rx_tick) >= 20) {
            char c = s_line_buf[0];
            if (c == 'F' || c == 'f' || c == 'B' || c == 'b' ||
                c == 'L' || c == 'l' || c == 'R' || c == 'r' ||
                c == 'S' || c == 's' || c == '1' || c == '0' ||
                c == 'T' || c == 't' || c == 'A' || c == 'a') {
                s_line_buf[1] = '\0';
                ProcessCommandLine(s_line_buf);
                s_line_idx = 0;
            }
        }
    }
    // 2. Multi-character command fallback if no trailing newline was sent (after 100ms idle)
    else if (s_line_idx > 1 && s_rx_tail == s_rx_head) {
        if ((HAL_GetTick() - s_last_rx_tick) >= 100) {
            s_line_buf[s_line_idx] = '\0';
            ProcessCommandLine(s_line_buf);
            s_line_idx = 0;
        }
    }
}

void Telemetry_SendData(float angle, float gyro, int16_t pwm,
                        int32_t enc_left, int32_t enc_right, uint8_t state)
{
    if (s_huart == NULL) return;

    const cascade_pid_t *pid = CascadePID_GetData();
    const motor_encoder_data_t *mot = MotorEncoder_GetData();

    char s_ang[16], s_gyr[16];
    char s_targ[16], s_vfilt[16], s_iterm[16];
    format_float(s_ang, sizeof(s_ang), angle, 2);
    format_float(s_gyr, sizeof(s_gyr), gyro, 2);
    format_float(s_targ, sizeof(s_targ), pid->target_angle_deg, 2);
    format_float(s_vfilt, sizeof(s_vfilt), pid->speed_filtered, 3);
    format_float(s_iterm, sizeof(s_iterm), pid->speed_ki * pid->speed_integral, 2);

    char tx_buf[160];
    int len = snprintf(tx_buf, sizeof(tx_buf),
                       "DATA:%s,%s,%d,%ld,%ld,%u\n"
                       "DBG:%s,%s,%s,%d,%d\r\n",
                       s_ang, s_gyr, (int)pwm, (long)enc_left, (long)enc_right, (unsigned int)state,
                       s_targ, s_vfilt, s_iterm, (int)mot->pwm_left, (int)mot->pwm_right);

    if (len > 0) {
        HAL_UART_Transmit(s_huart, (uint8_t*)tx_buf, (uint16_t)len, 15);
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
    char s_ang[16], s_spd[16];
    format_float(s_ang, sizeof(s_ang), angle, 2);
    format_float(s_spd, sizeof(s_spd), speed, 2);
    char buf[80];
    snprintf(buf, sizeof(buf), "STATUS:%s,%s,%u,BLE:%u\n",
             s_ang, s_spd, (unsigned int)state, (unsigned int)s_ble_connected);
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
