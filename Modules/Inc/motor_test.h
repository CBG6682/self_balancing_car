/**
 * ==============================================================================
 * @file    motor_test.h
 * @brief   Standalone Hardware Motor & Driver Diagnostic Test Module
 * ==============================================================================
 */

#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run one full automated test cycle for Left & Right GA25 Motors.
 *        Cycle steps:
 *        1. Left Motor Forward  (PWM +1500) for 1.2s -> Brake 0.5s
 *        2. Left Motor Reverse  (PWM -1500) for 1.2s -> Brake 0.5s
 *        3. Right Motor Forward (PWM +1500) for 1.2s -> Brake 0.5s
 *        4. Right Motor Reverse (PWM -1500) for 1.2s -> Brake 0.5s
 *        5. Both Motors Forward (PWM +1500) for 1.5s -> Brake 0.5s
 *        6. Both Motors Reverse (PWM -1500) for 1.5s -> Brake 1.0s
 */
void Motor_RunAutoTestLoop(void);

/**
 * @brief Manual motor test command with duration
 * @param pwm_l: Left PWM (-4199 to 4199)
 * @param pwm_r: Right PWM (-4199 to 4199)
 * @param duration_ms: Duration in milliseconds before stopping
 */
void Motor_ManualTest(int16_t pwm_l, int16_t pwm_r, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TEST_H */
