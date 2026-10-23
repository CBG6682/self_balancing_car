/**
 * ==============================================================================
 * @file    cascade_pid.c
 * @brief   Dual-Loop Cascade PID Controller Implementation.
 *          Target: STM32F401CCU6
 * ==============================================================================
 */

#include "cascade_pid.h"
#include <string.h>

/* Private module instance */
static cascade_pid_t s_pid;

/* -----------------------------------------------------------------------------
 * Public API Implementation
 * -------------------------------------------------------------------------- */

void CascadePID_Init(void)
{
    memset(&s_pid, 0, sizeof(cascade_pid_t));

    // Load initial gains from pid_AutoConfig.h
    s_pid.angle_kp = DEFAULT_ANGLE_KP;
    s_pid.angle_ki = DEFAULT_ANGLE_KI;
    s_pid.angle_kd = DEFAULT_ANGLE_KD;

    s_pid.speed_kp = DEFAULT_SPEED_KP;
    s_pid.speed_ki = DEFAULT_SPEED_KI;
    s_pid.speed_kd = DEFAULT_SPEED_KD;

    // Saturation and Anti-windup limits
    s_pid.max_pwm_limit         = PWM_MAX_LIMIT;
    s_pid.max_target_angle_deg  = MAX_TARGET_ANGLE_DEG;

    // Limit integral windup to prevent actuator saturation latency
    s_pid.angle_integral_limit  = 800.0f;   // Max PWM contribution from angle integral
    s_pid.speed_integral_limit  = 5.0f;     // Max degrees contribution from speed integral
}

void CascadePID_Reset(void)
{
    s_pid.angle_error    = 0.0f;
    s_pid.angle_integral = 0.0f;

    s_pid.speed_error    = 0.0f;
    s_pid.speed_integral = 0.0f;
    s_pid.speed_filtered = 0.0f;

    s_pid.target_angle_deg = 0.0f;
    s_pid.pwm_output_left  = 0;
    s_pid.pwm_output_right = 0;
}

void CascadePID_SetGains(float ang_kp, float ang_ki, float ang_kd, float spd_kp, float spd_ki)
{
    if (ang_kp >= 0.0f) s_pid.angle_kp = ang_kp;
    if (ang_ki >= 0.0f) s_pid.angle_ki = ang_ki;
    if (ang_kd >= 0.0f) s_pid.angle_kd = ang_kd;

    if (spd_kp >= 0.0f) s_pid.speed_kp = spd_kp;
    if (spd_ki >= 0.0f) s_pid.speed_ki = spd_ki;
}

void CascadePID_Compute(float target_speed, float measured_speed,
                        float measured_angle, float gyro_rate,
                        float turn_offset, float dt,
                        int16_t *out_pwm_left, int16_t *out_pwm_right)
{
    if (dt <= 0.0001f) {
        dt = 0.01f;
    }

    /* -------------------------------------------------------------------------
     * 1. OUTER LOOP: Velocity PI Control
     * Purpose: Compute desired pitch tilt angle to achieve target speed.
     * ---------------------------------------------------------------------- */
    // Apply low-pass filter to measured speed to suppress discrete encoder step noise
    s_pid.speed_filtered = 0.75f * s_pid.speed_filtered + 0.25f * measured_speed;

    s_pid.speed_error = target_speed - s_pid.speed_filtered;

    // Integral calculation with Anti-windup
    s_pid.speed_integral += s_pid.speed_error * dt;
    if (s_pid.speed_integral > s_pid.speed_integral_limit) {
        s_pid.speed_integral = s_pid.speed_integral_limit;
    } else if (s_pid.speed_integral < -s_pid.speed_integral_limit) {
        s_pid.speed_integral = -s_pid.speed_integral_limit;
    }

    // Desired pitch angle in degrees (outer loop output)
    // Note: To accelerate forward (positive speed), the robot must tilt slightly forward (positive angle)
    float raw_target_angle = (s_pid.speed_kp * s_pid.speed_error) + (s_pid.speed_ki * s_pid.speed_integral);

    // Clamp desired pitch angle to safe maximum (e.g. +/- 10 degrees)
    if (raw_target_angle > s_pid.max_target_angle_deg) {
        raw_target_angle = s_pid.max_target_angle_deg;
    } else if (raw_target_angle < -s_pid.max_target_angle_deg) {
        raw_target_angle = -s_pid.max_target_angle_deg;
    }
    s_pid.target_angle_deg = raw_target_angle;

    /* -------------------------------------------------------------------------
     * 2. INNER LOOP: Pitch Angle PD/PID Control
     * Purpose: High-speed balance stabilization maintaining target_angle.
     * When leaning forward (measured_angle > target_angle), motors must drive
     * forward (positive PWM) to push wheels under COM and restore balance.
     * ---------------------------------------------------------------------- */
    s_pid.angle_error = measured_angle - s_pid.target_angle_deg;

    // Angle integral accumulation with clamp
    s_pid.angle_integral += s_pid.angle_error * dt;
    if (s_pid.angle_integral > s_pid.angle_integral_limit) {
        s_pid.angle_integral = s_pid.angle_integral_limit;
    } else if (s_pid.angle_integral < -s_pid.angle_integral_limit) {
        s_pid.angle_integral = -s_pid.angle_integral_limit;
    }

    // Direct derivative from Gyroscope angular rate (+gyro_rate):
    // Positive pitch rate means tipping forward faster, requiring more forward torque
    float angle_derivative = gyro_rate;

    float balance_u = (s_pid.angle_kp * s_pid.angle_error) +
                      (s_pid.angle_ki * s_pid.angle_integral) +
                      (s_pid.angle_kd * angle_derivative);

    /* -------------------------------------------------------------------------
     * 3. Steering Distribution & Actuator Saturation Clamping
     * ---------------------------------------------------------------------- */
    float pwm_l = balance_u + turn_offset;
    float pwm_r = balance_u - turn_offset;

    if (pwm_l > (float)s_pid.max_pwm_limit)  pwm_l = (float)s_pid.max_pwm_limit;
    if (pwm_l < -(float)s_pid.max_pwm_limit) pwm_l = -(float)s_pid.max_pwm_limit;

    if (pwm_r > (float)s_pid.max_pwm_limit)  pwm_r = (float)s_pid.max_pwm_limit;
    if (pwm_r < -(float)s_pid.max_pwm_limit) pwm_r = -(float)s_pid.max_pwm_limit;

    s_pid.pwm_output_left  = (int16_t)pwm_l;
    s_pid.pwm_output_right = (int16_t)pwm_r;

    if (out_pwm_left != NULL) {
        *out_pwm_left = s_pid.pwm_output_left;
    }
    if (out_pwm_right != NULL) {
        *out_pwm_right = s_pid.pwm_output_right;
    }
}

float CascadePID_GetTargetAngle(void)
{
    return s_pid.target_angle_deg;
}

const cascade_pid_t* CascadePID_GetData(void)
{
    return &s_pid;
}
