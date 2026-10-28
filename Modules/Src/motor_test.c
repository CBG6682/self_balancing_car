/**
 * ==============================================================================
 * @file    motor_test.c
 * @brief   Standalone Hardware Motor & Driver Diagnostic Test Implementation
 * ==============================================================================
 */

#include "motor_test.h"
#include "motor_encoder.h"
#include "telemetry_uart.h"
#include "gpio.h"
#include <stdio.h>

#define TEST_PWM_SPEED   1500  // ~35% duty cycle (safe, gentle speed)
#define TEST_RUN_TIME_MS 1200  // 1.2 seconds run per phase
#define TEST_STOP_TIME_MS 500  // 0.5 seconds brake between phases

static void RunPhase(const char *phase_name, int16_t pwm_l, int16_t pwm_r, uint32_t duration_ms)
{
    char log_buf[100];
    const motor_encoder_data_t *enc = MotorEncoder_GetData();
    int32_t start_enc_l = enc->total_pulses_left;
    int32_t start_enc_r = enc->total_pulses_right;

    snprintf(log_buf, sizeof(log_buf), "\n[MOTOR_TEST] >>> %s (PWM_L=%d, PWM_R=%d)\n", phase_name, (int)pwm_l, (int)pwm_r);
    Telemetry_SendString(log_buf);

    Motor_Enable();
    Motor_SetOutput(pwm_l, pwm_r);

    // Run for duration while blinking LED and updating encoder readings
    uint32_t start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < duration_ms) {
        Board_LED_Toggle();
        MotorEncoder_Update(0.05f);
        HAL_Delay(50);
    }

    // Brake motor
    Motor_SetOutput(0, 0);
    Motor_Disable();

    int32_t delta_l = enc->total_pulses_left - start_enc_l;
    int32_t delta_r = enc->total_pulses_right - start_enc_r;
    snprintf(log_buf, sizeof(log_buf), "[MOTOR_TEST] DONE: Delta_EncL=%ld, Delta_EncR=%ld\n", (long)delta_l, (long)delta_r);
    Telemetry_SendString(log_buf);

    HAL_Delay(TEST_STOP_TIME_MS);
}

void Motor_RunAutoTestLoop(void)
{
    Telemetry_SendString("\n==================================================\n");
    Telemetry_SendString(" BAT DAU CHU TRINH KIEM TRA DONG CO GA25 + TB6612\n");
    Telemetry_SendString(" (Giu xe tren khong de 2 banh quay tu do)\n");
    Telemetry_SendString("==================================================\n");

    // Phase 1: Động cơ Trái quay TIẾN
    RunPhase("1/6. DONG CO TRAI - QUAY TIEN", +TEST_PWM_SPEED, 0, TEST_RUN_TIME_MS);

    // Phase 2: Động cơ Trái quay LÙI
    RunPhase("2/6. DONG CO TRAI - QUAY LUI", -TEST_PWM_SPEED, 0, TEST_RUN_TIME_MS);

    // Phase 3: Động cơ Phải quay TIẾN
    RunPhase("3/6. DONG CO PHAI - QUAY TIEN", 0, +TEST_PWM_SPEED, TEST_RUN_TIME_MS);

    // Phase 4: Động cơ Phải quay LÙI
    RunPhase("4/6. DONG CO PHAI - QUAY LUI", 0, -TEST_PWM_SPEED, TEST_RUN_TIME_MS);

    // Phase 5: Cả 2 Động cơ quay TIẾN
    RunPhase("5/6. CA HAI DONG CO - DONG THOI QUAY TIEN", +TEST_PWM_SPEED, +TEST_PWM_SPEED, 1500);

    // Phase 6: Cả 2 Động cơ quay LÙI
    RunPhase("6/6. CA HAI DONG CO - DONG THOI QUAY LUI", -TEST_PWM_SPEED, -TEST_PWM_SPEED, 1500);

    Telemetry_SendString("\n[MOTOR_TEST] HOAN THANH 1 VONG. NGHI 2 GIAY TRUOC KHI LAP LAI...\n");
    Board_LED_Set(0); // LED ON
    HAL_Delay(2000);
}

void Motor_ManualTest(int16_t pwm_l, int16_t pwm_r, uint32_t duration_ms)
{
    Motor_Enable();
    Motor_SetOutput(pwm_l, pwm_r);
    HAL_Delay(duration_ms);
    Motor_SetOutput(0, 0);
    Motor_Disable();
}
