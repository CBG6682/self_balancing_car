/**
 * ==============================================================================
 * @file    cascade_pid.c
 * @brief   Dual-Loop Cascade PID Controller Implementation.
 *          Target: STM32F401CCU6
 * ==============================================================================
 */

#include "cascade_pid.h"
#include <math.h>
#include <string.h>

/* Private module instance */
static cascade_pid_t s_pid;
static float s_gyro_filtered = 0.0f;
static float s_balance_offset = MECHANICAL_BALANCE_OFFSET_DEG;

/* -----------------------------------------------------------------------------
 * Public API Implementation
 * -------------------------------------------------------------------------- */

void CascadePID_Init(void)
{
    memset(&s_pid, 0, sizeof(cascade_pid_t));
    s_gyro_filtered = 0.0f;
    s_balance_offset = MECHANICAL_BALANCE_OFFSET_DEG;

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
    s_pid.angle_integral_limit  = 150.0f;   // Max PWM contribution from angle integral (ARR = 999 scale)
    s_pid.speed_integral_limit  = 3.0f;     // Max degrees contribution from speed integral
}

void CascadePID_Reset(void)
{
    s_pid.angle_error    = 0.0f;
    s_pid.angle_integral = 0.0f;

    s_pid.speed_error    = 0.0f;
    s_pid.speed_integral = 0.0f;
    s_pid.speed_filtered = 0.0f;
    s_gyro_filtered      = 0.0f;

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
    if (spd_ki >= 0.0f) {
        if (spd_ki != s_pid.speed_ki) {
            s_pid.speed_integral = 0.0f; // Bumpless transition when changing sKi live via Bluetooth
        }
        s_pid.speed_ki = spd_ki;
    }
}

void CascadePID_SetBalanceOffset(float offset_deg)
{
    if (offset_deg >= -15.0f && offset_deg <= 15.0f) {
        s_balance_offset = offset_deg;
    }
}

float CascadePID_GetBalanceOffset(void)
{
    return s_balance_offset;
}

void CascadePID_Compute(float target_speed, float measured_speed,
                        float measured_angle, float gyro_rate,
                        float turn_offset, float dt,
                        int16_t *out_pwm_left, int16_t *out_pwm_right)
{
    if (dt <= 0.0001f) {
        dt = 0.01f;
    }

    // Apply mechanical balance zero-point offset
    float angle_calibrated = measured_angle - s_balance_offset;

    /* -------------------------------------------------------------------------
     * 1. OUTER LOOP: Velocity PI Control (Virtual Mattress Compliance)
     * Purpose: Compute desired pitch tilt angle to brake drift & hold position.
     * ---------------------------------------------------------------------- */
    // Balanced low-pass filter on wheel speed (fc ~2.8Hz at 100Hz): cuts encoder noise with less phase lag
    s_pid.speed_filtered = 0.85f * s_pid.speed_filtered + 0.15f * measured_speed;

    // Smooth quadratic softening around zero velocity (+/- 0.02 m/s):
    // Eliminates hard deadband step cliff while suppressing 1-pulse encoder chatter at rest.
    float v_raw = s_pid.speed_filtered;
    float v_active = v_raw;
    if (fabsf(target_speed) < 0.002f) {
        float abs_v = fabsf(v_raw);
        if (abs_v < 0.02f) {
            v_active = v_raw * (abs_v / 0.02f);
        }
    }
    // P branch ALWAYS damps actual wheel speed (-v_active) so sKp=28 never whips the body on F/B commands;
    // I branch integrates (target_speed - v_raw) to smoothly glide the robot forward/backward at target_speed.
    s_pid.speed_error = -v_active;                 // cho P (luôn phanh êm & bù Back-EMF)
    float speed_error_i = target_speed - v_raw;    // cho I (dẫn tốc độ tiến/lùi & neo vị trí)

    // Position integral: active within +/- 6 deg so minor external pushes do not erase position anchor
    if (s_pid.speed_ki <= 0.001f) {
        s_pid.speed_integral = 0.0f;
    } else if (angle_calibrated >= -6.0f && angle_calibrated <= 6.0f) {
        s_pid.speed_integral += speed_error_i * dt;
        float i_lim = (s_pid.speed_ki > 0.01f) ? (2.5f / s_pid.speed_ki) : 3.0f; // Max +/-2.5 deg contribution
        if (s_pid.speed_integral > i_lim) {
            s_pid.speed_integral = i_lim;
        } else if (s_pid.speed_integral < -i_lim) {
            s_pid.speed_integral = -i_lim;
        }
    } else {
        // Gentle decay when tilted sharply to prevent sudden rubber-band lurch
        s_pid.speed_integral *= 0.98f;
    }

    // Desired pitch angle in degrees (outer loop output)
    float raw_target_angle = (s_pid.speed_kp * s_pid.speed_error) + (s_pid.speed_ki * s_pid.speed_integral);

    // Clamp desired pitch angle to safe maximum (e.g. +/- 5 degrees)
    if (raw_target_angle > s_pid.max_target_angle_deg) {
        raw_target_angle = s_pid.max_target_angle_deg;
    } else if (raw_target_angle < -s_pid.max_target_angle_deg) {
        raw_target_angle = -s_pid.max_target_angle_deg;
    }
    s_pid.target_angle_deg = raw_target_angle;

    /* -------------------------------------------------------------------------
     * 2. INNER LOOP: Pitch Angle PD/PID Control (Soft-Zone Mattress Damping)
     * Purpose: Soft compliance near upright 0 deg, full torque against external push.
     * ---------------------------------------------------------------------- */
    s_pid.angle_error = angle_calibrated - s_pid.target_angle_deg;

    // Angle integral accumulation with clamp
    s_pid.angle_integral += s_pid.angle_error * dt;
    if (s_pid.angle_integral > s_pid.angle_integral_limit) {
        s_pid.angle_integral = s_pid.angle_integral_limit;
    } else if (s_pid.angle_integral < -s_pid.angle_integral_limit) {
        s_pid.angle_integral = -s_pid.angle_integral_limit;
    }

    // Virtual Mattress Soft-Zone Gain Scheduling around equilibrium (|angle_error| < 1.2 deg):
    // Softens stiffness to 60% right at 0 deg (prevents hard-floor jitter & gear lash hammer),
    // ramping smoothly to 100% full Kp at >= 1.2 deg to resist external pushes cleanly.
    float abs_ang_err = fabsf(s_pid.angle_error);
    float kp_scale = 1.0f;
    if (abs_ang_err < 1.2f) {
        kp_scale = 0.60f + 0.40f * (abs_ang_err / 1.2f);
    }
    float eff_angle_kp = s_pid.angle_kp * kp_scale;

    // Continuous angular rate derivative from Gyroscope (fc ~10.6Hz at 100Hz):
    // Strong continuous damping acts as virtual foam shock absorber.
    s_gyro_filtered = 0.60f * s_gyro_filtered + 0.40f * gyro_rate;
    float angle_derivative = s_gyro_filtered;

    float balance_u = (eff_angle_kp * s_pid.angle_error) +
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
