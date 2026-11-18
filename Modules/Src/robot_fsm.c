/**
 * ==============================================================================
 * @file    robot_fsm.c
 * @brief   Finite State Machine (FSM) Implementation.
 *          Target: STM32F401CCU6
 * ==============================================================================
 */

#include "robot_fsm.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RECOVERY_TIMEOUT_TICKS  30      // 0.3 seconds at 100Hz (brief burst, avoid motor stall)
#define CMD_TIMEOUT_TICKS       50      // 0.5 second auto-brake when RC stops sending
#define SWINGUP_BURST_PWM       700     // High-torque pulse for self-righting (strictly <= 800, 80% load cap on ARR=999)
#define RC_MOVE_SPEED_MPS       0.15f   // Gentle manual driving speed (15 cm/s) so car never pitches & falls
#define RC_SPEED_RAMP_PER_TICK  0.004f  // Smooth target_speed ramp (0.40 m/s^2, ~0.37s soft launch/brake)
#define RC_TURN_OFFSET_PWM      150     // Differential PWM for turning (stable - kept unchanged)
#define OVERLOAD_TRIP_TICKS     10      // 10 ticks = 100ms sustained >= 80% PWM triggers shutdown
#define SAFE_STAND_TICKS_REQ    20      // 20 ticks = 200ms stable upright hold required to re-engage
#define PWM_SLEW_RATE_PER_TICK  40      // Smooth load ramp-up (~200ms from 0 to 800)

/* Private module state */
static robot_fsm_t s_fsm;

/* -----------------------------------------------------------------------------
 * Public API Implementation
 * -------------------------------------------------------------------------- */

void RobotFSM_Init(float theta_normal, float max_recovery_angle)
{
    memset(&s_fsm, 0, sizeof(robot_fsm_t));

    s_fsm.theta_safe_engage = THETA_SAFE_ENGAGE_DEG;
    s_fsm.theta_normal = (theta_normal > 0.0f) ? theta_normal : THETA_NORMAL_DEG;
    s_fsm.theta_max_recovery = (max_recovery_angle > 0.0f) ? max_recovery_angle : MAX_RECOVERY_ANGLE_DEG;

    s_fsm.state = STATE_IDLE;
    s_fsm.current_cmd = CMD_NONE;

    // Initially power down motors for safety
    Motor_Disable();
    CascadePID_Reset();
}

void RobotFSM_SetThresholds(float theta_normal, float max_recovery_angle)
{
    if (theta_normal > 0.0f) {
        s_fsm.theta_normal = theta_normal;
    }
    if (max_recovery_angle > theta_normal) {
        s_fsm.theta_max_recovery = max_recovery_angle;
    }
}

void RobotFSM_SetCommand(robot_cmd_t cmd)
{
    s_fsm.current_cmd = cmd;
    s_fsm.cmd_timeout_ticks = 0; // Reset watchdog timer on newly received command
}

void RobotFSM_Update(float angle, float gyro_rate, float speed, float dt,
                     int16_t *cmd_pwm_left, int16_t *cmd_pwm_right)
{
    float abs_angle = fabsf(angle);

    // Watchdog: If no command refreshed within 500ms, auto-stop to prevent runaway
    if (s_fsm.current_cmd != CMD_NONE && s_fsm.current_cmd != CMD_STOP) {
        s_fsm.cmd_timeout_ticks++;
        if (s_fsm.cmd_timeout_ticks > CMD_TIMEOUT_TICKS) {
            s_fsm.current_cmd = CMD_STOP;
        }
    }

    switch (s_fsm.state) {
        /* ---------------------------------------------------------------------
         * STATE_IDLE: Robot has fallen over or safety/overload shutdown triggered.
         * Action: Motors disabled, PWM 0, STBY LOW.
         * Transition: To BALANCING only when robot is upright in safe zone
         *             (|angle| <= theta_safe_engage) for >= 200ms continuously.
         * ------------------------------------------------------------------ */
        case STATE_IDLE:
            Motor_Disable();
            CascadePID_Reset();
            s_fsm.target_speed = 0.0f;
            s_fsm.turn_offset  = 0.0f;
            s_fsm.last_pwm_l   = 0;
            s_fsm.last_pwm_r   = 0;

            if (cmd_pwm_left != NULL)  *cmd_pwm_left = 0;
            if (cmd_pwm_right != NULL) *cmd_pwm_right = 0;

            // Self-wake only when held steady in the safe upright zone (|angle| <= 10 deg)
            if (abs_angle <= s_fsm.theta_safe_engage) {
                s_fsm.safe_stand_ticks++;
                if (s_fsm.safe_stand_ticks >= SAFE_STAND_TICKS_REQ) {
                    s_fsm.overload_tripped = 0;
                    s_fsm.overload_ticks   = 0;
                    Motor_Enable();
                    CascadePID_Reset();
                    MotorEncoder_ResetCounters();
                    s_fsm.state = STATE_BALANCING;
                }
            } else {
                s_fsm.safe_stand_ticks = 0;
            }
            break;

        /* ---------------------------------------------------------------------
         * STATE_BALANCING: Smooth Cascade PID balance control up to cutoff angle.
         * Action: Responds to Bluetooth commands or automatically holds position.
         * ------------------------------------------------------------------ */
        case STATE_BALANCING:
            // 1. Boundary & Safety Check: Emergency shutdown if past max cutoff angle
            if (abs_angle > s_fsm.theta_max_recovery) {
                s_fsm.state = STATE_IDLE;
                s_fsm.safe_stand_ticks = 0;
                Motor_Disable();
                CascadePID_Reset();
                if (cmd_pwm_left != NULL)  *cmd_pwm_left = 0;
                if (cmd_pwm_right != NULL) *cmd_pwm_right = 0;
                break;
            }

            // 2. Decode manual driving commands with smooth speed ramp for Forward/Backward
            float desired_speed = 0.0f;
            switch (s_fsm.current_cmd) {
                case CMD_FORWARD:
                    desired_speed     = +RC_MOVE_SPEED_MPS;
                    s_fsm.turn_offset = 0.0f;
                    break;
                case CMD_BACKWARD:
                    desired_speed     = -RC_MOVE_SPEED_MPS;
                    s_fsm.turn_offset = 0.0f;
                    break;
                case CMD_TURN_LEFT:
                    desired_speed     = 0.0f;
                    s_fsm.turn_offset = -RC_TURN_OFFSET_PWM;
                    break;
                case CMD_TURN_RIGHT:
                    desired_speed     = 0.0f;
                    s_fsm.turn_offset = +RC_TURN_OFFSET_PWM;
                    break;
                case CMD_STOP:
                case CMD_NONE:
                default:
                    desired_speed     = 0.0f;
                    s_fsm.turn_offset = 0.0f;
                    break;
            }

            // Smooth ramp target_speed to eliminate sudden forward/backward pitch whip
            float spd_diff = desired_speed - s_fsm.target_speed;
            if (spd_diff > RC_SPEED_RAMP_PER_TICK) {
                s_fsm.target_speed += RC_SPEED_RAMP_PER_TICK;
            } else if (spd_diff < -RC_SPEED_RAMP_PER_TICK) {
                s_fsm.target_speed -= RC_SPEED_RAMP_PER_TICK;
            } else {
                s_fsm.target_speed = desired_speed;
            }

            // 3. Compute Dual-Loop Cascade PID
            int16_t pid_out_l = 0;
            int16_t pid_out_r = 0;
            CascadePID_Compute(s_fsm.target_speed, speed, angle, gyro_rate,
                               s_fsm.turn_offset, dt, &pid_out_l, &pid_out_r);

            // 4. Motor Overload Protection: If demand hits >= 80% capacity (PWM_MAX_LIMIT)
            if (abs(pid_out_l) >= PWM_MAX_LIMIT || abs(pid_out_r) >= PWM_MAX_LIMIT) {
                s_fsm.overload_ticks++;
                if (s_fsm.overload_ticks >= OVERLOAD_TRIP_TICKS) {
                    // Motor overload detected (>80% load sustained for >= 100ms) -> Trip shutdown!
                    s_fsm.state = STATE_IDLE;
                    s_fsm.overload_tripped = 1;
                    s_fsm.safe_stand_ticks = 0;
                    Motor_Disable();
                    CascadePID_Reset();
                    if (cmd_pwm_left != NULL)  *cmd_pwm_left = 0;
                    if (cmd_pwm_right != NULL) *cmd_pwm_right = 0;
                    break;
                }
            } else {
                if (s_fsm.overload_ticks > 0) {
                    s_fsm.overload_ticks--;
                }
            }

            // 5. Smooth Load Ramp-Up (Slew Rate Limiting to prevent high torque jolts)
            int16_t diff_l = pid_out_l - s_fsm.last_pwm_l;
            if (diff_l > PWM_SLEW_RATE_PER_TICK) {
                pid_out_l = s_fsm.last_pwm_l + PWM_SLEW_RATE_PER_TICK;
            } else if (diff_l < -PWM_SLEW_RATE_PER_TICK) {
                pid_out_l = s_fsm.last_pwm_l - PWM_SLEW_RATE_PER_TICK;
            }

            int16_t diff_r = pid_out_r - s_fsm.last_pwm_r;
            if (diff_r > PWM_SLEW_RATE_PER_TICK) {
                pid_out_r = s_fsm.last_pwm_r + PWM_SLEW_RATE_PER_TICK;
            } else if (diff_r < -PWM_SLEW_RATE_PER_TICK) {
                pid_out_r = s_fsm.last_pwm_r - PWM_SLEW_RATE_PER_TICK;
            }

            s_fsm.last_pwm_l = pid_out_l;
            s_fsm.last_pwm_r = pid_out_r;

            if (cmd_pwm_left != NULL)  *cmd_pwm_left = pid_out_l;
            if (cmd_pwm_right != NULL) *cmd_pwm_right = pid_out_r;
            break;

        /* ---------------------------------------------------------------------
         * STATE_RECOVERY: Tilt is between theta_normal and max_recovery.
         * Action: Controlled burst pulse towards direction of fall.
         * ------------------------------------------------------------------ */
        case STATE_RECOVERY:
            s_fsm.recovery_timer_ticks++;

            // Check if successfully righted
            if (abs_angle <= s_fsm.theta_normal) {
                CascadePID_Reset(); // Clear integrator to avoid kick
                s_fsm.state = STATE_BALANCING;
                break;
            }

            // Check if recovery timed out (300ms) or tipped past boundary limit
            if (abs_angle > s_fsm.theta_max_recovery || s_fsm.recovery_timer_ticks >= RECOVERY_TIMEOUT_TICKS) {
                s_fsm.state = STATE_IDLE;
                s_fsm.safe_stand_ticks = 0;
                Motor_Disable();
                CascadePID_Reset();
                if (cmd_pwm_left != NULL)  *cmd_pwm_left = 0;
                if (cmd_pwm_right != NULL) *cmd_pwm_right = 0;
                break;
            }

            // Bang-bang acceleration pulse towards direction of fall (capped to SWINGUP_BURST_PWM <= 80%)
            int16_t burst_pwm = (angle > 0.0f) ? +SWINGUP_BURST_PWM : -SWINGUP_BURST_PWM;

            if (cmd_pwm_left != NULL)  *cmd_pwm_left = burst_pwm;
            if (cmd_pwm_right != NULL) *cmd_pwm_right = burst_pwm;
            break;

        default:
            s_fsm.state = STATE_IDLE;
            break;
    }
}

robot_state_t RobotFSM_GetState(void)
{
    return s_fsm.state;
}

const char* RobotFSM_GetStateString(void)
{
    switch (s_fsm.state) {
        case STATE_BALANCING: return "BALANCING";
        case STATE_RECOVERY:  return "RECOVERY";
        case STATE_IDLE:
        default:              return "IDLE";
    }
}

const robot_fsm_t* RobotFSM_GetData(void)
{
    return &s_fsm;
}
