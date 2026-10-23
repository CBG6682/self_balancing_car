/**
 * ==============================================================================
 * @file    pid_AutoConfig.h
 * @brief   Auto-generated PID & hardware control parameters.
 *          Optimized by Tools/auto_tune_pid.py (Nelder-Mead ITAE).
 * ==============================================================================
 */

#ifndef PID_AUTOCONFIG_H
#define PID_AUTOCONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 * Cascade PID Optimal Gains (Tuned via System Identification & ITAE)
 * -------------------------------------------------------------------------- */
#define DEFAULT_ANGLE_KP            101.35f
#define DEFAULT_ANGLE_KI            0.050f
#define DEFAULT_ANGLE_KD            2.688f

#define DEFAULT_SPEED_KP            8.00f
#define DEFAULT_SPEED_KI            0.500f
#define DEFAULT_SPEED_KD            0.000f

/* -----------------------------------------------------------------------------
 * Hardware & Actuator Limits
 * -------------------------------------------------------------------------- */
#define PWM_MAX_LIMIT               4199
#define PWM_DEADZONE_DEFAULT        450
#define ENCODER_CPR                 330
#define WHEEL_RADIUS_METERS         0.0325f
#define WHEEL_BASE_METERS           0.1600f

/* -----------------------------------------------------------------------------
 * FSM Thresholds & Loop Timing
 * -------------------------------------------------------------------------- */
#define CONTROL_DT_SEC              0.0100f
#define THETA_NORMAL_DEG            20.0f
#define MAX_RECOVERY_ANGLE_DEG      45.0f
#define MAX_TARGET_ANGLE_DEG        10.0f
#define MAX_TARGET_SPEED_MPS        0.80f

#ifdef __cplusplus
}
#endif

#endif /* PID_AUTOCONFIG_H */
