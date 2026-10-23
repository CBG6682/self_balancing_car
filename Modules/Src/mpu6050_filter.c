/**
 * ==============================================================================
 * @file    mpu6050_filter.c
 * @brief   MPU6050 IMU Driver & 1D Kalman Filter Implementation.
 *          Target: STM32F401CCU6 (I2C1 @ 400kHz, Fast Mode)
 * ==============================================================================
 */

#include "mpu6050_filter.h"
#include <math.h>
#include <string.h>

/* Private module state */
static I2C_HandleTypeDef *s_hi2c = NULL;
static mpu6050_data_t s_sensor_data;

/* -----------------------------------------------------------------------------
 * Private Helper Functions
 * -------------------------------------------------------------------------- */

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
    if (s_hi2c == NULL) return HAL_ERROR;
    return HAL_I2C_Mem_Write(s_hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

static HAL_StatusTypeDef MPU6050_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
    if (s_hi2c == NULL) return HAL_ERROR;
    return HAL_I2C_Mem_Read(s_hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100);
}

static void Kalman_Init(kalman_filter_t *kf)
{
    kf->q_angle = 0.001f;     // Process noise variance for angle
    kf->q_bias = 0.003f;      // Process noise variance for gyro bias
    kf->r_measure = 0.03f;    // Measurement noise variance (accelerometer)
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->rate = 0.0f;

    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

static float Kalman_Update(kalman_filter_t *kf, float new_angle, float new_rate, float dt)
{
    // Predict step:
    // Update angle estimate using rate and elapsed time
    kf->rate = new_rate - kf->bias;
    kf->angle += dt * kf->rate;

    // Update estimation error covariance
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->q_bias * dt;

    // Measurement update step:
    // Innovation (measurement residual)
    float y = new_angle - kf->angle;

    // Innovation covariance
    float S = kf->P[0][0] + kf->r_measure;

    // Optimal Kalman gain
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Update state with measurement
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    // Update error covariance
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/* -----------------------------------------------------------------------------
 * Public API Implementation
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return HAL_ERROR;
    s_hi2c = hi2c;

    memset(&s_sensor_data, 0, sizeof(mpu6050_data_t));
    Kalman_Init(&s_sensor_data.kalman);

    // 1. Verify Device ID via WHO_AM_I
    uint8_t who_am_i = 0;
    if (MPU6050_ReadRegisters(MPU6050_REG_WHO_AM_I, &who_am_i, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        return HAL_ERROR;
    }

    // 2. Reset Device and Wake Up (Clear SLEEP bit, select PLL with X-axis Gyro clock)
    if (MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10);

    // 3. Set Sample Rate Divider: SMPLRT_DIV = 0 (Sample Rate = 1kHz)
    if (MPU6050_WriteRegister(MPU6050_REG_SMPLRT_DIV, 0x00) != HAL_OK) {
        return HAL_ERROR;
    }

    // 4. Digital Low Pass Filter: DLPF_CFG = 3 (Accel BW = 44Hz, Gyro BW = 42Hz)
    // Minimizes high frequency vibration noise from GA25 gearbox
    if (MPU6050_WriteRegister(MPU6050_REG_CONFIG, 0x03) != HAL_OK) {
        return HAL_ERROR;
    }

    // 5. Gyroscope Full Scale: +/-2000 deg/s (FS_SEL = 3, Scale = 16.4 LSB/dps)
    if (MPU6050_WriteRegister(MPU6050_REG_GYRO_CONFIG, 0x18) != HAL_OK) {
        return HAL_ERROR;
    }

    // 6. Accelerometer Full Scale: +/-4g (AFS_SEL = 1, Scale = 8192 LSB/g)
    if (MPU6050_WriteRegister(MPU6050_REG_ACCEL_CONFIG, 0x08) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void MPU6050_Calibrate(void)
{
    const uint16_t CALIBRATION_SAMPLES = 300;
    float sum_gyro_pitch = 0.0f;
    float sum_accel_pitch = 0.0f;
    uint8_t raw_buffer[14];

    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (MPU6050_ReadRegisters(MPU6050_REG_ACCEL_XOUT_H, raw_buffer, 14) == HAL_OK) {
            int16_t ax_raw = (int16_t)((raw_buffer[0] << 8) | raw_buffer[1]);
            int16_t ay_raw = (int16_t)((raw_buffer[2] << 8) | raw_buffer[3]);
            int16_t az_raw = (int16_t)((raw_buffer[4] << 8) | raw_buffer[5]);
            int16_t gx_raw = (int16_t)((raw_buffer[8] << 8) | raw_buffer[9]);

            float ax = (float)ax_raw / MPU6050_ACCEL_SENS_4G;
            float ay = (float)ay_raw / MPU6050_ACCEL_SENS_4G;
            float az = (float)az_raw / MPU6050_ACCEL_SENS_4G;
            float gx = (float)gx_raw / MPU6050_GYRO_SENS_2000DPS;

            // Pitch angle from accelerometer: atan2(ay, sqrt(ax^2 + az^2))
            float pitch_acc = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;

            sum_gyro_pitch += gx;
            sum_accel_pitch += pitch_acc;
        }
        HAL_Delay(2);
    }

    s_sensor_data.gyro_pitch_offset = sum_gyro_pitch / (float)CALIBRATION_SAMPLES;
    s_sensor_data.accel_pitch_offset = sum_accel_pitch / (float)CALIBRATION_SAMPLES;

    // Seed Kalman filter angle with current baseline accelerometer angle
    s_sensor_data.kalman.angle = 0.0f;
    s_sensor_data.kalman.bias = 0.0f;
    s_sensor_data.is_calibrated = 1;
}

void MPU6050_Update(float dt)
{
    uint8_t raw_buffer[14];

    if (MPU6050_ReadRegisters(MPU6050_REG_ACCEL_XOUT_H, raw_buffer, 14) != HAL_OK) {
        return; // Retain previous valid estimate on transient I2C error
    }

    int16_t ax_raw = (int16_t)((raw_buffer[0] << 8) | raw_buffer[1]);
    int16_t ay_raw = (int16_t)((raw_buffer[2] << 8) | raw_buffer[3]);
    int16_t az_raw = (int16_t)((raw_buffer[4] << 8) | raw_buffer[5]);

    int16_t gx_raw = (int16_t)((raw_buffer[8]  << 8) | raw_buffer[9]);
    int16_t gy_raw = (int16_t)((raw_buffer[10] << 8) | raw_buffer[11]);
    int16_t gz_raw = (int16_t)((raw_buffer[12] << 8) | raw_buffer[13]);

    s_sensor_data.ax = (float)ax_raw / MPU6050_ACCEL_SENS_4G;
    s_sensor_data.ay = (float)ay_raw / MPU6050_ACCEL_SENS_4G;
    s_sensor_data.az = (float)az_raw / MPU6050_ACCEL_SENS_4G;

    s_sensor_data.gx = (float)gx_raw / MPU6050_GYRO_SENS_2000DPS;
    s_sensor_data.gy = (float)gy_raw / MPU6050_GYRO_SENS_2000DPS;
    s_sensor_data.gz = (float)gz_raw / MPU6050_GYRO_SENS_2000DPS;

    // Pitch angle from Accelerometer (with calibrated zero offset)
    float raw_accel_pitch = atan2f(s_sensor_data.ay, sqrtf(s_sensor_data.ax * s_sensor_data.ax + s_sensor_data.az * s_sensor_data.az)) * RAD_TO_DEG;
    float accel_pitch = raw_accel_pitch - s_sensor_data.accel_pitch_offset;

    // Pitch angular rate from Gyroscope (with zero bias subtracted)
    float gyro_pitch_rate = s_sensor_data.gx - s_sensor_data.gyro_pitch_offset;

    // 1D Kalman Filter Fusion
    s_sensor_data.pitch = Kalman_Update(&s_sensor_data.kalman, accel_pitch, gyro_pitch_rate, dt);
    s_sensor_data.gyro_rate = s_sensor_data.kalman.rate;
}

float MPU6050_GetPitch(void)
{
    return s_sensor_data.pitch;
}

float MPU6050_GetGyroRate(void)
{
    return s_sensor_data.gyro_rate;
}

const mpu6050_data_t* MPU6050_GetData(void)
{
    return &s_sensor_data;
}
