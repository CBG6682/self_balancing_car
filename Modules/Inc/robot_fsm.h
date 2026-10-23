/**
 * ==============================================================================
 * @file    robot_fsm.h
 * @brief   Finite State Machine (FSM) for Self-Balancing Robot.
 *          States:
 *          - STATE_BALANCING: Normal upright cascade control & RC commands
 *          - STATE_RECOVERY: Bang-bang swing-up impulse on moderate tilt
 *          - STATE_IDLE: Safe shutdown when fallen over (PB0 STBY LOW, PWM 0)
 *          Target: STM32F401CCU6
 * ==============================================================================
 */

#ifndef ROBOT_FSM_H
#define ROBOT_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid_AutoConfig.h"
#include "cascade_pid.h"
#include "motor_encoder.h"

/* -----------------------------------------------------------------------------
 * Types and Enums
 * -------------------------------------------------------------------------- */
typedef enum {
    STATE_IDLE = 0,             // Fallen over or disabled, TB6612 powered down
    STATE_BALANCING = 1,        // Stable balance, responds to RC commands or holds position
    STATE_RECOVERY = 2          // Automatic swing-up impulse recovery
} robot_state_t;

typedef enum {
    CMD_NONE = 0,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_STOP
} robot_cmd_t;

typedef struct {
    robot_state_t state;
    robot_cmd_t current_cmd;
    float target_speed;             // Speed target sent to Cascade PID (m/s)
    float turn_offset;              // Steering PWM offset sent to Cascade PID
    uint16_t recovery_timer_ticks;  // Timeout counter in RECOVERY state (10ms ticks)
    uint16_t cmd_timeout_ticks;     // Inactivity timeout counter for RC commands
    float theta_normal;             // Normal balance threshold (e.g. 20.0 deg)
    float theta_max_recovery;       // Boundary limit for recovery attempt (e.g. 45.0 deg)
} robot_fsm_t;

/* -----------------------------------------------------------------------------
 * Public API Prototypes
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialize FSM with angle thresholds.
 * @param  theta_normal: Threshold for normal balancing in degrees (typically 20.0)
 * @param  max_recovery_angle: Maximum tilt allowed for recovery attempt (typically 45.0)
 */
void RobotFSM_Init(float theta_normal, float max_recovery_angle);

/**
 * @brief  Set remote control command (from Bluetooth or serial script).
 * @param  cmd: Command enum (FORWARD, BACKWARD, LEFT, RIGHT, STOP)
 */
void RobotFSM_SetCommand(robot_cmd_t cmd);

/**
 * @brief  Update state machine logic, transitions and execute control.
 *         Must be invoked periodically every dt = 0.01s (100Hz loop).
 * @param  angle: Filtered pitch angle in degrees
 * @param  gyro_rate: Pitch angular velocity in deg/s
 * @param  speed: Average forward velocity in m/s
 * @param  dt: Time delta in seconds (typically 0.01f)
 * @param  cmd_pwm_left: Pointer to left wheel PWM output
 * @param  cmd_pwm_right: Pointer to right wheel PWM output
 */
void RobotFSM_Update(float angle, float gyro_rate, float speed, float dt,
                     int16_t *cmd_pwm_left, int16_t *cmd_pwm_right);

/**
 * @brief  Get current FSM state enum.
 * @retval robot_state_t enum value
 */
robot_state_t RobotFSM_GetState(void);

/**
 * @brief  Get human-readable string representation of current state.
 * @retval String: "IDLE", "BALANCING", or "RECOVERY"
 */
const char* RobotFSM_GetStateString(void);

/**
 * @brief  Get read-only pointer to FSM data structure.
 * @retval Pointer to constant robot_fsm_t
 */
const robot_fsm_t* RobotFSM_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_FSM_H */
