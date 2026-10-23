/**
 * ==============================================================================
 * @file    motor_encoder.c
 * @brief   TB6612FNG Dual H-Bridge Motor Driver & 2-Wheel Encoder Implementation.
 *          Target: STM32F401CCU6
 * ==============================================================================
 */

#include "motor_encoder.h"
#include <string.h>

#define PI_CONST                3.14159265358979323846f

/* Inversion polarity flags in case motor/encoder wires are swapped */
#define MOTOR_LEFT_POLARITY     (+1)
#define MOTOR_RIGHT_POLARITY    (+1)
#define ENCODER_LEFT_POLARITY   (+1)
#define ENCODER_RIGHT_POLARITY  (+1)

/* Private module state */
static TIM_HandleTypeDef *s_htim_pwm   = NULL;
static TIM_HandleTypeDef *s_htim_enc_l = NULL;
static TIM_HandleTypeDef *s_htim_enc_r = NULL;

static motor_encoder_data_t s_motor_data;
static uint16_t s_last_cnt_l = 0;
static uint16_t s_last_cnt_r = 0;

/* -----------------------------------------------------------------------------
 * Private Helper Functions
 * -------------------------------------------------------------------------- */

/**
 * @brief Maps commanded PWM to overcome static motor deadzone linearly.
 *        PWM_out = deadzone + (|cmd| * (PWM_MAX - deadzone)) / PWM_MAX
 */
static uint16_t ApplyDeadzone(int16_t cmd_pwm, uint16_t deadzone)
{
    if (cmd_pwm == 0) {
        return 0;
    }

    int32_t abs_pwm = (cmd_pwm > 0) ? cmd_pwm : -cmd_pwm;
    if (abs_pwm > PWM_MAX_LIMIT) {
        abs_pwm = PWM_MAX_LIMIT;
    }

    int32_t compensated = (int32_t)deadzone + ((abs_pwm * (int32_t)(PWM_MAX_LIMIT - deadzone)) / PWM_MAX_LIMIT);
    if (compensated > PWM_MAX_LIMIT) {
        compensated = PWM_MAX_LIMIT;
    }

    return (uint16_t)compensated;
}

/* -----------------------------------------------------------------------------
 * Public API Implementation
 * -------------------------------------------------------------------------- */

void MotorEncoder_Init(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_enc_l, TIM_HandleTypeDef *htim_enc_r)
{
    s_htim_pwm   = htim_pwm;
    s_htim_enc_l = htim_enc_l;
    s_htim_enc_r = htim_enc_r;

    memset(&s_motor_data, 0, sizeof(motor_encoder_data_t));
    s_motor_data.deadzone = PWM_DEADZONE_DEFAULT;

    // 1. Start Encoder Counters (TIM2 and TIM3)
    if (s_htim_enc_l != NULL) {
        HAL_TIM_Encoder_Start(s_htim_enc_l, TIM_CHANNEL_ALL);
        s_last_cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_l);
    }
    if (s_htim_enc_r != NULL) {
        HAL_TIM_Encoder_Start(s_htim_enc_r, TIM_CHANNEL_ALL);
        s_last_cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_r);
    }

    // 2. Start PWM Output Channels (TIM4 CH3 & CH4)
    if (s_htim_pwm != NULL) {
        HAL_TIM_PWM_Start(s_htim_pwm, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(s_htim_pwm, TIM_CHANNEL_4);
        __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_4, 0);
    }

    // 3. Enable TB6612 H-Bridge driver
    Motor_Enable();
}

void MotorEncoder_Update(float dt)
{
    if (dt <= 0.0001f || s_htim_enc_l == NULL || s_htim_enc_r == NULL) {
        return;
    }

    // Read current 16-bit timer counters
    uint16_t curr_cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_l);
    uint16_t curr_cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_r);

    // Compute signed 16-bit difference to automatically handle 0 <-> 65535 roll-over
    int16_t delta_l = (int16_t)(curr_cnt_l - s_last_cnt_l) * ENCODER_LEFT_POLARITY;
    int16_t delta_r = (int16_t)(curr_cnt_r - s_last_cnt_r) * ENCODER_RIGHT_POLARITY;

    s_last_cnt_l = curr_cnt_l;
    s_last_cnt_r = curr_cnt_r;

    s_motor_data.delta_pulses_left  = delta_l;
    s_motor_data.delta_pulses_right = delta_r;

    s_motor_data.total_pulses_left  += delta_l;
    s_motor_data.total_pulses_right += delta_r;

    // Conversion factor from pulses to linear distance (meters)
    // Distance per revolution = 2 * PI * R
    // Distance per pulse = (2 * PI * R) / ENCODER_CPR
    float meters_per_pulse = (2.0f * PI_CONST * WHEEL_RADIUS_METERS) / (float)ENCODER_CPR;

    // Linear wheel speeds (m/s)
    s_motor_data.speed_left_mps  = ((float)delta_l * meters_per_pulse) / dt;
    s_motor_data.speed_right_mps = ((float)delta_r * meters_per_pulse) / dt;

    // Average body forward velocity
    s_motor_data.speed_avg_mps   = (s_motor_data.speed_left_mps + s_motor_data.speed_right_mps) * 0.5f;
    s_motor_data.distance_m     += s_motor_data.speed_avg_mps * dt;
}

void Motor_SetOutput(int16_t left_pwm, int16_t right_pwm)
{
    if (!s_motor_data.is_enabled || s_htim_pwm == NULL) {
        if (s_htim_pwm != NULL) {
            __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_4, 0);
        }
        s_motor_data.pwm_left  = 0;
        s_motor_data.pwm_right = 0;
        return;
    }

    // Apply polarity
    left_pwm  *= MOTOR_LEFT_POLARITY;
    right_pwm *= MOTOR_RIGHT_POLARITY;

    // Saturate input commands
    if (left_pwm > PWM_MAX_LIMIT)   left_pwm = PWM_MAX_LIMIT;
    if (left_pwm < -PWM_MAX_LIMIT)  left_pwm = -PWM_MAX_LIMIT;
    if (right_pwm > PWM_MAX_LIMIT)  right_pwm = PWM_MAX_LIMIT;
    if (right_pwm < -PWM_MAX_LIMIT) right_pwm = -PWM_MAX_LIMIT;

    s_motor_data.pwm_left  = left_pwm;
    s_motor_data.pwm_right = right_pwm;

    // 1. Left Motor Direction (AIN1 = PA4, AIN2 = PA5)
    if (left_pwm > 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (left_pwm < 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }

    // 2. Right Motor Direction (BIN1 = PA6, BIN2 = PA7)
    if (right_pwm > 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else if (right_pwm < 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }

    // 3. Set PWM Duty Cycle with Deadzone Compensation (TIM4 CH3 & CH4)
    uint16_t ccr_left  = ApplyDeadzone(left_pwm, s_motor_data.deadzone);
    uint16_t ccr_right = ApplyDeadzone(right_pwm, s_motor_data.deadzone);

    __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_3, ccr_left);
    __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_4, ccr_right);
}

void Motor_Disable(void)
{
    s_motor_data.is_enabled = 0;

    // Shut down PWM immediately
    if (s_htim_pwm != NULL) {
        __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_4, 0);
    }

    // Put H-Bridge in short brake / high-Z
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

    // Pull STBY LOW (PB0) to power down TB6612 output FETs
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    s_motor_data.pwm_left  = 0;
    s_motor_data.pwm_right = 0;
}

void Motor_Enable(void)
{
    // Pull STBY HIGH (PB0)
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    s_motor_data.is_enabled = 1;
}

void Motor_SetDeadzone(uint16_t deadzone)
{
    if (deadzone > 1500) {
        deadzone = 1500;
    }
    s_motor_data.deadzone = deadzone;
}

void MotorEncoder_ResetCounters(void)
{
    s_motor_data.total_pulses_left  = 0;
    s_motor_data.total_pulses_right = 0;
    s_motor_data.delta_pulses_left  = 0;
    s_motor_data.delta_pulses_right = 0;
    s_motor_data.distance_m         = 0.0f;
    s_motor_data.speed_left_mps     = 0.0f;
    s_motor_data.speed_right_mps    = 0.0f;
    s_motor_data.speed_avg_mps      = 0.0f;

    if (s_htim_enc_l != NULL) {
        s_last_cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_l);
    }
    if (s_htim_enc_r != NULL) {
        s_last_cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_r);
    }
}

const motor_encoder_data_t* MotorEncoder_GetData(void)
{
    return &s_motor_data;
}
