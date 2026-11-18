/**
 * ==============================================================================
 * @file    motor_encoder.h
 * @brief   TB6612FNG Dual H-Bridge Motor Driver & 2-Wheel Encoder Module.
 *          Target: STM32F401CCU6
 *          - Motor PWM: TIM4_CH3 (PB8), TIM4_CH4 (PB9), ARR = 4199
 *          - Direction: PA4 (AIN1), PA5 (AIN2), PA3 (BIN1), PA7 (BIN2), PA2 (STBY)
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
 * @brief  Get current motor deadzone compensation value.
 */
uint16_t Motor_GetDeadzone(void);

/**
 * @brief  Directly output raw PWM duty cycle without deadzone compensation.
 *         Used for automated deadzone calibration and low-level testing.
 */
void Motor_SetRawPWM(int16_t pwm_l, int16_t pwm_r);

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

typedef enum {
    SYS_MODE_BIN_PROBE = 0,  // Static probe mode: STBY=0V, PWM=0, logic & candidate pins=3.3V
    SYS_MODE_OFF       = 1,  // Complete shutdown: ALL pins strictly 0V, auto-balance disabled
    SYS_MODE_AUTO      = 2,  // Active self-balancing mode: Cascade PID + FSM active
    SYS_MODE_MANUAL    = 3   // Direct manual motor control (M:FWD / M:REV)
} system_mode_t;

extern volatile system_mode_t g_system_mode;
extern volatile uint8_t g_motor_manual_override;

/**
 * @brief  Enter Static Multimeter Probe Mode:
 *         Lock STBY (PA2) strictly to 0V, drive AIN1, AIN2, BIN1, BIN2, and PWM (PWMA, PWMB) HIGH (3.3V)
 *         so user can verify hardware wiring with a multimeter while motors remain safely disabled.
 */
void Motor_EnterMode_BIN_Probe(void);

/**
 * @brief  Selectively probe driver pins with STBY strictly locked to 0V.
 * @param  a_en: 1 to set AIN1 (PA4) & AIN2 (PA5) = 3.3V
 * @param  b_en: 1 to set BIN1 (PA3) & BIN2 (PA7) = 3.3V
 * @param  pwm_en: 1 to set PWMA (PB8) & PWMB (PB9) = 3.3V (100% duty)
 */
void Motor_ProbeDriverPins(uint8_t a_en, uint8_t b_en, uint8_t pwm_en);

/**
 * @brief  Enter OFF Mode: Lock ALL pins (STBY, driver pins, PWM, and candidate pins) strictly to 0V.
 *         Prevents auto-waking into self-balancing when placed on the ground.
 */
void Motor_EnterMode_OFF(void);

/**
 * @brief  Enter AUTO Self-Balancing Mode (Explicitly requested via 'AUTO' or 'MODE:AUTO').
 */
void Motor_EnterMode_AUTO(void);

/**
 * @brief  Force continuous DC 3.3V on motor control pins for static VOM multimeter hardware probing.
 * @param  enable: 1 to hold STBY=3.3V, AIN1=3.3V, BIN1=3.3V, PWMA=3.3V (100%), PWMB=3.3V (100%)
 *                 0 to release and turn off all outputs.
 */
void Motor_ForceDCOutput(uint8_t enable);

/**
 * @brief  Force continuous DC 3.3V on motor control pins for static VOM multimeter hardware probing.
 * @param  dir: +1 for Forward (AIN1=3.3V, AIN2=0V, BIN1=3.3V, BIN2=0V)
 *              -1 for Reverse (AIN1=0V, AIN2=3.3V, BIN1=0V, BIN2=3.3V)
 *               0 for OFF (all 0V).
 */
void Motor_ForceDCDirection(int8_t dir);

/**
 * @brief  Drive ALL available motor control pins HIGH (3.3V) for static multimeter probing.
 * @param  stby_high: 1 to also set STBY (PA2) = 3.3V (all 7 pins 3.3V),
 *                    0 to keep STBY (PA2) = 0V (6 pins 3.3V, driver locked in standby).
 */
void Motor_ProbeAllPins_High(uint8_t stby_high);

/**
 * @brief  Lock STBY to 0V (TB6612 driver safely disabled) and drive all control pins HIGH (3.3V)
 *         for static multimeter hardware probing.
 */
void Motor_ProbeBIN_High_STBY_Lock(void);

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
