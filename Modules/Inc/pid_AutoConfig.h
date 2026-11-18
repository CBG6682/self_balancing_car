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
 * Cascade PID Optimal Gains (ARR = 999 Scale, User-Verified Stable Tuning)
 * -------------------------------------------------------------------------- */
#define DEFAULT_ANGLE_KP            40.00f  // Điểm ổn định nhất người dùng đã xác nhận (mềm quanh 0 còn ~24)
#define DEFAULT_ANGLE_KI            0.000f
#define DEFAULT_ANGLE_KD            1.400f  // Giảm chấn mượt, đứng im không rung

#define DEFAULT_SPEED_KP            28.00f  // Vượt ngưỡng Back-EMF, phanh gọn
#define DEFAULT_SPEED_KI            2.500f  // Giữ vị trí & dẫn tốc độ tiến/lùi êm
#define DEFAULT_SPEED_KD            0.000f

/* -----------------------------------------------------------------------------
 * Hardware & Actuator Limits (ARR = 999, 21kHz, 80% Ceiling Protection)
 * -------------------------------------------------------------------------- */
#define PWM_TIMER_ARR               999
#define MOTOR_MAX_LOAD_PERCENT      80
#define PWM_MAX_LIMIT               800     // (1000 * 80) / 100 = 800 (80% maximum allowable load)

// Deadzone riêng từng bánh (~85% ngưỡng có tải: Trái 27*0.85=23, Phải 20*0.85=17)
#define PWM_DEADZONE_LEFT           23
#define PWM_DEADZONE_RIGHT          17
#define PWM_DEADZONE_DEFAULT        20      // Giá trị tham chiếu chung nếu gọi API cũ

// Bù lệch tốc độ giữa 2 động cơ (đo thực tế 7.2s cho R/L=93% ở 0.88 -> nâng lên 0.93 cho khớp ~100%)
#define MOTOR_LEFT_SCALE            1.00f
#define MOTOR_RIGHT_SCALE           0.93f

#define ENCODER_CPR                 937     // GA25 11 xung * tỷ số 21.3 * 4 cạnh = 937.2 xung/vòng
#define WHEEL_RADIUS_METERS         0.0325f
#define WHEEL_BASE_METERS           0.1600f

/* -----------------------------------------------------------------------------
 * FSM Thresholds & Loop Timing
 * -------------------------------------------------------------------------- */
#define CONTROL_DT_SEC              0.0100f
#define THETA_SAFE_ENGAGE_DEG       10.0f   // Must return within +/-10 deg to re-engage
#define THETA_NORMAL_DEG            18.0f   // Normal balancing envelope (+/-18 deg)
#define MAX_RECOVERY_ANGLE_DEG      25.0f   // Cutoff angle (+/-25 deg, prevents stall overload)
#define MAX_TARGET_ANGLE_DEG        12.0f   // Đủ biên độ cho sKp=28 bù Back-EMF khi xe chạy tiến/lùi êm
#define MAX_TARGET_SPEED_MPS        0.50f

/* -----------------------------------------------------------------------------
 * Motor & Encoder Direction Polarities (+1 or -1) & Balance Offset
 * -------------------------------------------------------------------------- */
#define MOTOR_LEFT_POLARITY         (+1)
#define MOTOR_RIGHT_POLARITY        (-1)
#define ENCODER_LEFT_POLARITY       (-1)
#define ENCODER_RIGHT_POLARITY      (+1)
#define MECHANICAL_BALANCE_OFFSET_DEG (-3.60f) // Balance offset set to -3.60 deg per user request

#ifdef __cplusplus
}
#endif

#endif /* PID_AUTOCONFIG_H */
