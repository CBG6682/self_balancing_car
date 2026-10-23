/**
 * ==============================================================================
 * @file    robot_fsm.c
 * @brief   Finite State Machine (FSM) Implementation.
 *          Target: STM32F401CCU6
 * ==============================================================================
 */

#include "robot_fsm.h"
#include <math.h>
#include <string.h>

#define RECOVERY_TIMEOUT_TICKS  150     // 1.5 seconds at 100Hz (10ms * 150 = 1500ms)
#define CMD_TIMEOUT_TICKS       50      // 0.5 second auto-brake when RC stops sending
#define SWINGUP_BURST_PWM       3800    // High-torque acceleration pulse for self-righting
#define RC_MOVE_SPEED_MPS       0.35f   // Default manual driving speed (m/s)
#define RC_TURN_OFFSET_PWM      600     // Differential PWM for turning

/* Private module state */
static robot_fsm_t s_fsm;

/* -----------------------------------------------------------------------------
 * Public API Implementation
 * -------------------------------------------------------------------------- */

void RobotFSM_Init(float theta_normal, float max_recovery_angle)
{
    memset(&s_fsm, 0, sizeof(robot_fsm_t));

    s_fsm.theta_normal = (theta_normal > 0.0f) ? theta_normal : THETA_NORMAL_DEG;
    s_fsm.theta_max_recovery = (max_recovery_angle > 0.0f) ? max_recovery_angle : MAX_RECOVERY_ANGLE_DEG;

    s_fsm.state = STATE_IDLE;
    s_fsm.current_cmd = CMD_NONE;

    // Initially power down motors for safety
    Motor_Disable();
    CascadePID_Reset();
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
         * STATE_IDLE: Robot has fallen over or safety shutdown triggered.
         * Action: Motors disabled, PWM 0, STBY LOW.
         * Transition: To BALANCING when robot is picked upright (|angle| <= theta_normal).
         * ------------------------------------------------------------------ */
        case STATE_IDLE:
            Motor_Disable();
            CascadePID_Reset();
            s_fsm.target_speed = 0.0f;
            s_fsm.turn_offset  = 0.0f;

            if (cmd_pwm_left != NULL)  *cmd_pwm_left = 0;
            if (cmd_pwm_right != NULL) *cmd_pwm_right = 0;

            // Self-wake when user places robot upright
            if (abs_angle <= s_fsm.theta_normal) {
                Motor_Enable();
                CascadePID_Reset();
                MotorEncoder_ResetCounters();
                s_fsm.state = STATE_BALANCING;
            }
            break;

        /* ---------------------------------------------------------------------
         * STATE_BALANCING: Normal upright cascade control.
         * Action: Responds to Bluetooth commands or automatically holds position.
         * ------------------------------------------------------------------ */
        case STATE_BALANCING:
            // 1. Check boundary conditions
            if (abs_angle > s_fsm.theta_max_recovery) {
                // Tipped over completely -> Emergency shutdown
                s_fsm.state = STATE_IDLE;
                Motor_Disable();
                CascadePID_Reset();
                if (cmd_pwm_left != NULL)  *cmd_pwm_left = 0;
                if (cmd_pwm_right != NULL) *cmd_pwm_right = 0;
                break;
            }
            else if (abs_angle > s_fsm.theta_normal) {
                // Tilted beyond normal threshold -> Trigger recovery swing-up
                s_fsm.state = STATE_RECOVERY;
                s_fsm.recovery_timer_ticks = 0;
                break;
            }

            // 2. Decode manual driving commands
            switch (s_fsm.current_cmd) {
                case CMD_FORWARD:
                    s_fsm.target_speed = +RC_MOVE_SPEED_MPS;
                    s_fsm.turn_offset  = 0.0f;
                    break;
                case CMD_BACKWARD:
                    s_fsm.target_speed = -RC_MOVE_SPEED_MPS;
                    s_fsm.turn_offset  = 0.0f;
                    break;
                case CMD_TURN_LEFT:
                    s_fsm.target_speed = 0.0f;
                    s_fsm.turn_offset  = -RC_TURN_OFFSET_PWM;
                    break;
                case CMD_TURN_RIGHT:
                    s_fsm.target_speed = 0.0f;
                    s_fsm.turn_offset  = +RC_TURN_OFFSET_PWM;
                    break;
                case CMD_STOP:
                case CMD_NONE:
                default:
                    // Auto-brake: lock target velocity to 0 m/s to stay stationary in place
                    s_fsm.target_speed = 0.0f;
                    s_fsm.turn_offset  = 0.0f;
                    break;
            }

            // 3. Compute Dual-Loop Cascade PID
            CascadePID_Compute(s_fsm.target_speed, speed, angle, gyro_rate,
                               s_fsm.turn_offset, dt, cmd_pwm_left, cmd_pwm_right);
            break;

        /* ---------------------------------------------------------------------
         * STATE_RECOVERY: Tilt is between theta_normal and max_recovery.
         * Action: High acceleration burst in tilt direction to pitch robot upright.
         * ------------------------------------------------------------------ */
        case STATE_RECOVERY:
            s_fsm.recovery_timer_ticks++;

            // Check if successfully righted
            if (abs_angle <= s_fsm.theta_normal) {
                CascadePID_Reset(); // Clear integrator to avoid kick
                s_fsm.state = STATE_BALANCING;
                break;
            }

            // Check if recovery timed out (1.5s) or tipped past boundary limit
            if (abs_angle > s_fsm.theta_max_recovery || s_fsm.recovery_timer_ticks >= RECOVERY_TIMEOUT_TICKS) {
                s_fsm.state = STATE_IDLE;
                Motor_Disable();
                CascadePID_Reset();
                if (cmd_pwm_left != NULL)  *cmd_pwm_left = 0;
                if (cmd_pwm_right != NULL) *cmd_pwm_right = 0;
                break;
            }

            // Bang-bang acceleration pulse towards direction of fall
            // Moves wheels under the center of mass to bring torso upright
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
