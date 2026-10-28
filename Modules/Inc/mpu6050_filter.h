/**
 * ==============================================================================
 * @file    mpu6050_filter.h
 * @brief   MPU6050 IMU Driver & 1D Kalman Filter for Inverted Pendulum Robot.
 *          Target: STM32F401CCU6 (I2C1 @ 400kHz, Fast Mode)
 * ==============================================================================
 */

#ifndef MPU6050_FILTER_H
#define MPU6050_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid_AutoConfig.h"

#ifndef MECHANICAL_BALANCE_OFFSET_DEG
#define MECHANICAL_BALANCE_OFFSET_DEG (0.0f)
#endif

/* -----------------------------------------------------------------------------
 * MPU6050 I2C Configuration
 * -------------------------------------------------------------------------- */
#define MPU6050_I2C_ADDR            (0x68 << 1) // AD0 pin tied to GND

/* Registers */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

/* Expected WHO_AM_I value */
#define MPU6050_WHO_AM_I_VALUE      0x68

/* Sensor Sensitivity Scales */
#define MPU6050_ACCEL_SENS_4G       8192.0f     // LSB/g for +/-4g
#define MPU6050_GYRO_SENS_2000DPS   16.4f       // LSB/(deg/s) for +/-2000dps
#define RAD_TO_DEG                  57.2957795f
#define DEG_TO_RAD                  0.01745329f

/* -----------------------------------------------------------------------------
 * Data Structures
 * -------------------------------------------------------------------------- */
typedef struct {
    float q_angle;      // Process noise variance for angle
    float q_bias;       // Process noise variance for gyro bias
    float r_measure;    // Measurement noise variance
    float angle;        // Filtered angle (degrees)
    float bias;         // Estimated gyro bias (deg/s)
    float rate;         // Unbiased angular velocity (deg/s)
    float P[2][2];      // Error covariance matrix
} kalman_filter_t;

typedef struct {
    float pitch;                // Estimated pitch angle (degrees)
    float gyro_rate;            // Angular rate along pitch axis (deg/s)
    float ax, ay, az;           // Acceleration in g
    float gx, gy, gz;           // Angular rates in deg/s
    float gyro_pitch_offset;    // Calibrated zero bias
    float accel_pitch_offset;   // Calibrated zero tilt offset
    uint8_t is_calibrated;      // Calibration completed flag
    kalman_filter_t kalman;     // 1D Kalman Filter instance
} mpu6050_data_t;

/* -----------------------------------------------------------------------------
 * Public API Prototypes
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialize MPU6050 over I2C1 (wake up, 42Hz DLPF, +/-2000dps, +/-4g).
 * @param  hi2c: Pointer to HAL I2C handle (e.g. &hi2c1)
 * @retval HAL status (HAL_OK on success, HAL_ERROR if not responding)
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Calibrate zero-rate gyro bias and baseline accelerometer offset.
 *         Should be called when the robot is completely stationary.
 */
void MPU6050_Calibrate(void);

/**
 * @brief  Read sensor data burst and update Kalman filter estimate.
 *         Must be called periodically in the 100Hz control loop (dt = 0.01s).
 * @param  dt: Elapsed sampling time in seconds (typically 0.01f)
 */
void MPU6050_Update(float dt);

/**
 * @brief  Get current filtered pitch angle in degrees.
 * @retval Pitch angle (-90.0 to +90.0 degrees)
 */
float MPU6050_GetPitch(void);

/**
 * @brief  Get current unbiased angular rate along pitch axis in deg/s.
 * @retval Angular rate (deg/s)
 */
float MPU6050_GetGyroRate(void);

/**
 * @brief  Hardware I2C bus recovery routine (toggles SCL 9 times if SDA stuck LOW).
 *         Call before MX_I2C1_Init() or before MPU6050_Init().
 */
void MPU6050_I2C_BusClear(void);

/**
 * @brief  Check if MPU6050 successfully initialized and communication is active.
 * @retval 1 if connected and responding, 0 otherwise
 */
uint8_t MPU6050_IsConnected(void);

/**
 * @brief  Set mechanical balance pitch zero offset in degrees.
 * @param  offset_deg: Pitch offset (subtracted from raw accelerometer pitch)
 */
void MPU6050_SetZeroOffset(float offset_deg);

/**
 * @brief  Get pointer to the full MPU6050 data structure.
 * @retval Pointer to constant mpu6050_data_t
 */
const mpu6050_data_t* MPU6050_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_FILTER_H */
