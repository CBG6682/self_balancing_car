/**
 * ==============================================================================
 * @file    motor_encoder.h
 * @brief   TB6612FNG Dual H-Bridge Motor Driver & 2-Wheel Encoder Module.
 *          Target: STM32F401CCU6
 *          - Motor PWM: TIM4_CH3 (PB8), TIM4_CH4 (PB9), ARR = 4199
 *          - Direction: PA4 (AIN1), PA5 (AIN2), PA6 (BIN1), PA7 (BIN2), PB0 (STBY)
 *          - Encoder Left: TIM2 (PA0/PA1), ARR = 65535
 *          - Encoder Right: TIM3 (PB4/PB5), ARR = 65535
 * ==============================================================================
 */

#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid_AutoConfig.h"

/* -----------------------------------------------------------------------------
 * Data Structures
 * -------------------------------------------------------------------------- */
typedef struct {
    int32_t total_pulses_left;      // Cumulative encoder pulses (Left)
    int32_t total_pulses_right;     // Cumulative encoder pulses (Right)
    int16_t delta_pulses_left;      // Pulses in current control cycle (10ms)
    int16_t delta_pulses_right;     // Pulses in current control cycle (10ms)
    float speed_left_mps;           // Real linear speed of left wheel (m/s)
    float speed_right_mps;          // Real linear speed of right wheel (m/s)
    float speed_avg_mps;            // Average linear speed of robot (m/s)
    float distance_m;               // Total traveled distance (meters)
    int16_t pwm_left;               // Target command PWM (-4199 to +4199)
    int16_t pwm_right;              // Target command PWM (-4199 to +4199)
    uint16_t deadzone;              // Static friction deadzone compensation
    uint8_t is_enabled;             // Driver enabled state (STBY pin HIGH)
    int8_t motor_left_polarity;     // Motor left direction (+1 or -1)
    int8_t motor_right_polarity;    // Motor right direction (+1 or -1)
    int8_t enc_left_polarity;       // Encoder left direction (+1 or -1)
    int8_t enc_right_polarity;      // Encoder right direction (+1 or -1)
} motor_encoder_data_t;

/* -----------------------------------------------------------------------------
 * Public API Prototypes
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialize timers for PWM output, Encoders and TB6612 standby GPIO.
 * @param  htim_pwm: Pointer to PWM timer handle (&htim4)
 * @param  htim_enc_l: Pointer to left encoder timer handle (&htim2)
 * @param  htim_enc_r: Pointer to right encoder timer handle (&htim3)
 */
void MotorEncoder_Init(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_enc_l, TIM_HandleTypeDef *htim_enc_r);

/**
 * @brief  Sample encoder counters, handle 16-bit wrap-around and compute linear velocities.
 *         Must be invoked periodically every dt = 0.01s (100Hz loop).
 * @param  dt: Time delta in seconds (typically 0.01f)
 */
void MotorEncoder_Update(float dt);

/**
 * @brief  Set motor speed and direction with automatic deadzone compensation.
 * @param  left_pwm: Motor left command (-PWM_MAX_LIMIT to +PWM_MAX_LIMIT)
 * @param  right_pwm: Motor right command (-PWM_MAX_LIMIT to +PWM_MAX_LIMIT)
 */
void Motor_SetOutput(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief  Safely disable motors (set PWM to 0, pull STBY LOW).
 */
void Motor_Disable(void);

/**
 * @brief  Enable motor driver (pull STBY HIGH).
 */
void Motor_Enable(void);

/**
 * @brief  Adjust static friction deadzone compensation threshold.
 * @param  deadzone: Deadzone offset (typically 300 to 600 for GA25 motors)
 */
void Motor_SetDeadzone(uint16_t deadzone);

/**
 * @brief  Configure motor and encoder direction polarities (+1 or -1).
 */
void Motor_SetPolarity(int8_t ml, int8_t mr, int8_t el, int8_t er);

/**
 * @brief  Get current motor and encoder direction polarities.
 */
void Motor_GetPolarity(int8_t *ml, int8_t *mr, int8_t *el, int8_t *er);

/**
 * @brief  Run motors directly for hardware testing / verification.
 */
void Motor_TestRun(int16_t pwm_l, int16_t pwm_r);

/**
 * @brief  Reset cumulative encoder pulse counters and distance to zero.
 */
void MotorEncoder_ResetCounters(void);

/**
 * @brief  Get read-only pointer to module telemetry data.
 * @retval Pointer to constant motor_encoder_data_t
 */
const motor_encoder_data_t* MotorEncoder_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ENCODER_H */
