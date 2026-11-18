/**
 * ==============================================================================
 * @file    motor_encoder.c
 * @brief   TB6612FNG Dual H-Bridge Motor Driver & 2-Wheel Encoder Implementation.
 *          Target: STM32F401CCU6
 * ==============================================================================
 */

#include "motor_encoder.h"
#include "cascade_pid.h"
#include "robot_fsm.h"
#include <string.h>

#define PI_CONST                3.14159265358979323846f

/* Inversion polarity flags fallback from pid_AutoConfig.h if not defined */
#ifndef MOTOR_LEFT_POLARITY
#define MOTOR_LEFT_POLARITY     (+1)
#endif
#ifndef MOTOR_RIGHT_POLARITY
#define MOTOR_RIGHT_POLARITY    (-1)
#endif
#ifndef ENCODER_LEFT_POLARITY
#define ENCODER_LEFT_POLARITY   (+1)
#endif
#ifndef ENCODER_RIGHT_POLARITY
#define ENCODER_RIGHT_POLARITY  (-1)
#endif

/* Global manual override flag to bypass FSM when probing hardware */
volatile uint8_t g_motor_manual_override = 0;

/* Private module state */
static TIM_HandleTypeDef *s_htim_pwm   = NULL;
static TIM_HandleTypeDef *s_htim_enc_l = NULL;
static TIM_HandleTypeDef *s_htim_enc_r = NULL;

static motor_encoder_data_t s_motor_data;
static uint16_t s_last_cnt_l = 0;
static uint16_t s_last_cnt_r = 0;
static uint16_t s_deadzone_left  = PWM_DEADZONE_LEFT;
static uint16_t s_deadzone_right = PWM_DEADZONE_RIGHT;

/* -----------------------------------------------------------------------------
 * Private Helper Functions
 * -------------------------------------------------------------------------- */

/**
 * @brief Maps commanded PWM to overcome static motor deadzone linearly.
 *        PWM_out = deadzone + (|cmd| * (PWM_MAX - deadzone)) / PWM_MAX
 */
#define DEADZONE_SMOOTH_THRESHOLD   10  // Ramp mềm quanh 0 để mô phỏng độ êm của đệm (tránh giật răng hộp số)

/**
 * @brief Maps commanded PWM to overcome static motor deadzone linearly with smooth zero-transition.
 *        Avoids chattering/jittering when robot is near upright balance point.
 */
static uint16_t ApplyDeadzone(int16_t cmd_pwm, uint16_t deadzone)
{
    int32_t abs_pwm = (cmd_pwm > 0) ? cmd_pwm : -cmd_pwm;
    if (abs_pwm <= 2) {
        return 0; // Vùng chết thực sự chống rung lắc tại điểm cân bằng
    }
    if (abs_pwm > PWM_MAX_LIMIT) {
        abs_pwm = PWM_MAX_LIMIT;
    }

    int32_t compensated;
    if (abs_pwm < DEADZONE_SMOOTH_THRESHOLD) {
        // Continuous smooth ramp from 0 up to deadzone to eliminate limit-cycle chattering
        compensated = (abs_pwm * (int32_t)deadzone) / DEADZONE_SMOOTH_THRESHOLD;
    } else {
        compensated = (int32_t)deadzone + (((abs_pwm - DEADZONE_SMOOTH_THRESHOLD) * (int32_t)(PWM_MAX_LIMIT - deadzone)) / (PWM_MAX_LIMIT - DEADZONE_SMOOTH_THRESHOLD));
    }

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
    s_deadzone_left  = PWM_DEADZONE_LEFT;
    s_deadzone_right = PWM_DEADZONE_RIGHT;
    s_motor_data.motor_left_polarity  = MOTOR_LEFT_POLARITY;
    s_motor_data.motor_right_polarity = MOTOR_RIGHT_POLARITY;
    s_motor_data.enc_left_polarity    = ENCODER_LEFT_POLARITY;
    s_motor_data.enc_right_polarity   = ENCODER_RIGHT_POLARITY;

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
    int16_t delta_l = (int16_t)(curr_cnt_l - s_last_cnt_l) * s_motor_data.enc_left_polarity;
    int16_t delta_r = (int16_t)(curr_cnt_r - s_last_cnt_r) * s_motor_data.enc_right_polarity;

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

    // 1. Nhân hệ số cân bằng lực giữa 2 bánh + đảo chiều theo polarity
    int32_t scaled_l = (int32_t)((float)left_pwm  * MOTOR_LEFT_SCALE)  * s_motor_data.motor_left_polarity;
    int32_t scaled_r = (int32_t)((float)right_pwm * MOTOR_RIGHT_SCALE) * s_motor_data.motor_right_polarity;

    // 2. Giới hạn bão hòa
    if (scaled_l > PWM_MAX_LIMIT)   scaled_l = PWM_MAX_LIMIT;
    if (scaled_l < -PWM_MAX_LIMIT)  scaled_l = -PWM_MAX_LIMIT;
    if (scaled_r > PWM_MAX_LIMIT)   scaled_r = PWM_MAX_LIMIT;
    if (scaled_r < -PWM_MAX_LIMIT)  scaled_r = -PWM_MAX_LIMIT;

    left_pwm  = (int16_t)scaled_l;
    right_pwm = (int16_t)scaled_r;

    s_motor_data.pwm_left  = left_pwm;
    s_motor_data.pwm_right = right_pwm;

    // 3. Điều khiển chiều bánh trái (ngưỡng +/- 2; vùng chết kích hoạt Short-Brake tạo ma sát đệm ảo)
    if (left_pwm > 2) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (left_pwm < -2) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    } else {
        // TB6612 Short-Brake (IN1=H, IN2=H): nối tắt cuộn dây tạo lực cản lăn điện từ thụ động giống mặt đệm
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }

    // 4. Điều khiển chiều bánh phải (vùng chết kích hoạt Short-Brake tạo ma sát đệm ảo)
    if (right_pwm > 2) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else if (right_pwm < -2) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else {
        // TB6612 Short-Brake (IN1=H, IN2=H): tạo lực cản lăn thụ động quanh điểm cân bằng
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }

    // 5. Xuất xung PWM với Deadzone RIÊNG từng bánh
    uint16_t ccr_left  = ApplyDeadzone(left_pwm,  s_deadzone_left);
    uint16_t ccr_right = ApplyDeadzone(right_pwm, s_deadzone_right);

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

    // Pull STBY LOW (PA2) to power down TB6612 output FETs
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    s_motor_data.pwm_left  = 0;
    s_motor_data.pwm_right = 0;
}

void Motor_Enable(void)
{
    // Pull STBY HIGH (PA2)
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    s_motor_data.is_enabled = 1;
}

void Motor_SetDeadzone(uint16_t deadzone)
{
    if (deadzone > 400) {
        deadzone = 400;
    }
    s_motor_data.deadzone = deadzone;
    if (deadzone == 0) {
        s_deadzone_left  = 0;
        s_deadzone_right = 0;
    } else {
        int32_t delta = (int32_t)deadzone - (int32_t)PWM_DEADZONE_DEFAULT;
        int32_t dz_l  = (int32_t)PWM_DEADZONE_LEFT  + delta;
        int32_t dz_r  = (int32_t)PWM_DEADZONE_RIGHT + delta;
        if (dz_l < 0) dz_l = 0;
        if (dz_r < 0) dz_r = 0;
        if (dz_l > 400) dz_l = 400;
        if (dz_r > 400) dz_r = 400;
        s_deadzone_left  = (uint16_t)dz_l;
        s_deadzone_right = (uint16_t)dz_r;
    }
}

uint16_t Motor_GetDeadzone(void)
{
    return s_motor_data.deadzone;
}

void Motor_SetRawPWM(int16_t left_pwm, int16_t right_pwm)
{
    if (s_htim_pwm == NULL) return;

    g_motor_manual_override = 1;
    Motor_Enable();

    // Apply polarity
    left_pwm  *= s_motor_data.motor_left_polarity;
    right_pwm *= s_motor_data.motor_right_polarity;

    // Saturate input commands
    if (left_pwm > PWM_MAX_LIMIT)   left_pwm = PWM_MAX_LIMIT;
    if (left_pwm < -PWM_MAX_LIMIT)  left_pwm = -PWM_MAX_LIMIT;
    if (right_pwm > PWM_MAX_LIMIT)  right_pwm = PWM_MAX_LIMIT;
    if (right_pwm < -PWM_MAX_LIMIT) right_pwm = -PWM_MAX_LIMIT;

    s_motor_data.pwm_left  = left_pwm;
    s_motor_data.pwm_right = right_pwm;

    // Left direction
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

    // Right direction
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

    // Direct CCR output without deadzone curve
    uint16_t ccr_l = (uint16_t)((left_pwm < 0) ? -left_pwm : left_pwm);
    uint16_t ccr_r = (uint16_t)((right_pwm < 0) ? -right_pwm : right_pwm);

    __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_3, ccr_l);
    __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_4, ccr_r);
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

void Motor_SetPolarity(int8_t ml, int8_t mr, int8_t el, int8_t er)
{
    if (ml == 1 || ml == -1) s_motor_data.motor_left_polarity = ml;
    if (mr == 1 || mr == -1) s_motor_data.motor_right_polarity = mr;
    if (el == 1 || el == -1) s_motor_data.enc_left_polarity = el;
    if (er == 1 || er == -1) s_motor_data.enc_right_polarity = er;
}

void Motor_GetPolarity(int8_t *ml, int8_t *mr, int8_t *el, int8_t *er)
{
    if (ml) *ml = s_motor_data.motor_left_polarity;
    if (mr) *mr = s_motor_data.motor_right_polarity;
    if (el) *el = s_motor_data.enc_left_polarity;
    if (er) *er = s_motor_data.enc_right_polarity;
}

void Motor_ForceDCOutput(uint8_t enable)
{
    Motor_ForceDCDirection(enable ? 1 : 0);
}

void Motor_ForceDCDirection(int8_t dir)
{
    if (dir != 0) {
        g_motor_manual_override = 1;
        Motor_Enable(); // PA2 (STBY) HIGH -> 3.3V solid DC

        if (dir > 0) {
            // Forward: AIN1=1, AIN2=0 (Left FWD); BIN1=1, BIN2=0 (Right FWD)
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        } else {
            // Reverse: AIN1=0, AIN2=1 (Left REV); BIN1=0, BIN2=1 (Right REV)
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        }

        // Set 80% maximum PWM duty cycle (PWM_MAX_LIMIT = 3359) on PB8 (PWMA) & PB9 (PWMB)
        if (s_htim_pwm != NULL) {
            __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_3, PWM_MAX_LIMIT);
            __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_4, PWM_MAX_LIMIT);
        }
    } else {
        Motor_EnterMode_OFF();
    }
}

volatile system_mode_t g_system_mode = SYS_MODE_AUTO;

void Motor_ProbeDriverPins(uint8_t a_en, uint8_t b_en, uint8_t pwm_en)
{
    g_system_mode = SYS_MODE_BIN_PROBE;
    g_motor_manual_override = 1;

    // 1. STBY pin (PA2): STRICTLY LOCKED to 0.0V (LOW) -> TB6612 powered down / output FETs high-Z
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    // 2. Set PWM channels: 100% duty (ARR = 999) -> Constant 3.3V DC on PB8 (PWMA) and PB9 (PWMB)
    if (s_htim_pwm != NULL) {
        uint16_t ccr = pwm_en ? PWM_TIMER_ARR : 0;
        __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_3, ccr);
        __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_4, ccr);
    }

    // 3. Driver direction pins -> Set according to enable flags
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, a_en ? GPIO_PIN_SET : GPIO_PIN_RESET); // PA4 (AIN1)
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, a_en ? GPIO_PIN_SET : GPIO_PIN_RESET); // PA5 (AIN2)
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, b_en ? GPIO_PIN_SET : GPIO_PIN_RESET); // PA3 (BIN1)
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, b_en ? GPIO_PIN_SET : GPIO_PIN_RESET); // PA7 (BIN2)

    s_motor_data.is_enabled = 0;
    s_motor_data.pwm_left   = 0;
    s_motor_data.pwm_right  = 0;
}

void Motor_EnterMode_BIN_Probe(void)
{
    // Default probe mode: All AIN, BIN, and PWM (PWMA/PWMB) are driven HIGH (3.3V), STBY strictly LOCKED to 0V
    Motor_ProbeDriverPins(1, 1, 1);
}

void Motor_EnterMode_OFF(void)
{
    g_system_mode = SYS_MODE_OFF;
    g_motor_manual_override = 1;

    // Lock ALL pins strictly to 0.0V
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);     // PA2 = 0V
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);     // PA4 = 0V
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);     // PA5 = 0V
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);     // PA3 = 0V (New BIN1)
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);     // PA7 = 0V

    if (s_htim_pwm != NULL) {
        __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_3, 0);          // PB8 = 0V
        __HAL_TIM_SET_COMPARE(s_htim_pwm, TIM_CHANNEL_4, 0);          // PB9 = 0V
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_RESET); // PA6, PA8 = 0V
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 |
                             GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    Motor_Disable();
    CascadePID_Reset();
}

void Motor_EnterMode_AUTO(void)
{
    g_system_mode = SYS_MODE_AUTO;
    g_motor_manual_override = 0;

    // Reset candidate pins to LOW
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 |
                             GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    Motor_Disable();
    CascadePID_Reset();
    RobotFSM_Init(THETA_NORMAL_DEG, MAX_RECOVERY_ANGLE_DEG);
}

void Motor_ProbeAllPins_High(uint8_t stby_high)
{
    if (stby_high) {
        Motor_EnterMode_BIN_Probe();
        HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    } else {
        Motor_EnterMode_BIN_Probe();
    }
}

void Motor_ProbeBIN_High_STBY_Lock(void)
{
    Motor_EnterMode_BIN_Probe();
}

void Motor_TestRun(int16_t pwm_l, int16_t pwm_r)
{
    g_motor_manual_override = 1;
    Motor_Enable();
    Motor_SetOutput(pwm_l, pwm_r);
}

const motor_encoder_data_t* MotorEncoder_GetData(void)
{
    return &s_motor_data;
}
