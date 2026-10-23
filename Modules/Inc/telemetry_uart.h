/**
 * ==============================================================================
 * @file    telemetry_uart.h
 * @brief   Telemetry Communication & Command Protocol over USART1 (115200bps).
 *          Target: STM32F401CCU6 (PA9 = TX, PA10 = RX)
 *          Supports:
 *          - Handshake: PING / PONG
 *          - Hot PID tuning: SET_PID:<ang_kp>,<ang_ki>,<ang_kd>,<spd_kp>,<spd_ki>
 *          - Motion control: FORWARD, BACKWARD, LEFT, RIGHT, STOP
 *          - Telemetry streaming: DATA:<angle>,<gyro>,<pwm>,<enc_l>,<enc_r>,<state>
 * ==============================================================================
 */

#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "robot_fsm.h"
#include "cascade_pid.h"

#define TELEMETRY_RX_BUF_SIZE       256
#define TELEMETRY_CMD_LINE_SIZE     128

/* -----------------------------------------------------------------------------
 * Public API Prototypes
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialize UART telemetry module and start non-blocking RX interrupt.
 * @param  huart: Pointer to UART handle (e.g. &huart1)
 */
void Telemetry_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Start or re-arm interrupt-driven byte reception.
 */
void Telemetry_StartReceiveIT(void);

/**
 * @brief  Interrupt service callback when a new byte arrives on USART1.
 *         Must be invoked from HAL_UART_RxCpltCallback or USART1_IRQHandler.
 * @param  byte: Received character
 */
void Telemetry_OnRxByte(uint8_t byte);

/**
 * @brief  Process incoming command buffer (invoked in main loop, non-blocking).
 */
void Telemetry_ProcessIncoming(void);

/**
 * @brief  Transmit telemetry data packet over UART.
 *         Format: DATA:<angle>,<gyro>,<pwm>,<enc_left>,<enc_right>,<state>\n
 * @param  angle: Filtered pitch angle in degrees
 * @param  gyro: Pitch angular rate in deg/s
 * @param  pwm: Commanded PWM average
 * @param  enc_left: Cumulative pulses left wheel
 * @param  enc_right: Cumulative pulses right wheel
 * @param  state: Current FSM state integer (0=IDLE, 1=BALANCING, 2=RECOVERY)
 */
void Telemetry_SendData(float angle, float gyro, int16_t pwm,
                        int32_t enc_left, int32_t enc_right, uint8_t state);

/**
 * @brief  Transmit raw string message over UART.
 * @param  str: Null-terminated string
 */
void Telemetry_SendString(const char *str);

/**
 * @brief  Check if telemetry streaming mode is enabled.
 * @retval 1 if active, 0 if disabled
 */
uint8_t Telemetry_IsStreaming(void);

/**
 * @brief  Enable or disable periodic telemetry stream.
 * @param  enable: 1 to start streaming, 0 to stop
 */
void Telemetry_SetStreaming(uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_UART_H */
