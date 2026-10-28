/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050_filter.h"
#include "motor_encoder.h"
#include "cascade_pid.h"
#include "robot_fsm.h"
#include "telemetry_uart.h"
#include "pid_AutoConfig.h"
#include "motor_test.h"
#include "uart_loopback_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ==============================================================================
 * CHẾ ĐỘ CHẨN ĐOÁN & KIỂM TRA PHẦN CỨNG:
 *   RUN_UART_LOOPBACK_TEST:
 *     1: KÍCH HOẠT CHẾ ĐỘ LOOPBACK TEST CỔNG UART (PA9 nối PA10 bằng 1 sợi dây cắm)
 *        - STM32 tự phát PING, nếu toàn bộ tín hiệu nhận về là PING -> ĐÈN LED PC13 CHỚP LIÊN TỤC.
 *        - Rút dây ra -> ĐÈN LED TẮT NGÚM.
 *     0: Tắt chế độ test loopback (để xe chạy tự cân bằng bình thường).
 *
 *   RUN_MOTOR_TEST_MODE:
 *     1: Chế độ kiểm tra động cơ tự động 6 pha
 *     0: Tắt
 * ============================================================================== */
#define RUN_UART_LOOPBACK_TEST 1
#define RUN_MOTOR_TEST_MODE    0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t g_control_tick_10ms_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Relocate Interrupt Vector Table to match WeAct HID Bootloader offset (0x08004000) */
  SCB->VTOR = 0x08004000;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize Robot Control Modules */
  Board_LED_Init();
  CascadePID_Init();
  MotorEncoder_Init(&htim4, &htim2, &htim3);
  RobotFSM_Init(THETA_NORMAL_DEG, MAX_RECOVERY_ANGLE_DEG);
  Telemetry_Init(&huart1);

#if (RUN_UART_LOOPBACK_TEST == 1)
  /* Chạy chế độ Loopback Test cổng UART1 (PA9 nối PA10): Không cần cảm biến hay động cơ */
  UART_Loopback_Test_Run();
#endif

  /* Clear I2C bus to unfreeze MPU6050 in case of previous warm reset */
  MPU6050_I2C_BusClear();
  MX_I2C1_Init();

  /* Initialize and calibrate MPU6050 IMU */
  if (MPU6050_Init(&hi2c1) == HAL_OK)
  {
    Telemetry_SendString("[SYS] MPU6050: OK (WHO_AM_I verified)\n");
    MPU6050_Calibrate();
    Telemetry_SendString("[SYS] IMU Calibration: OK (Kalman ready)\n");
  }
  else
  {
    Telemetry_SendString("[ERR] MPU6050 Init FAILED! Check PB6/PB7 wiring & VCC\n");
  }

  /* Start 100Hz Control Loop Timer (10ms tick interrupt) */
  HAL_TIM_Base_Start_IT(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if (RUN_MOTOR_TEST_MODE == 1)
    /* Chế độ kiểm tra động cơ tự động: Lặp lại chu trình 6 pha */
    Motor_RunAutoTestLoop();
#else
    /* Continuously process incoming UART packets for lowest command latency */
    Telemetry_ProcessIncoming();

    if (g_control_tick_10ms_flag)
    {
      g_control_tick_10ms_flag = 0;

      // 2. Read MPU6050 raw data and update Kalman filter estimate
      MPU6050_Update(CONTROL_DT_SEC);

      // 3. Read Encoders and calculate wheel velocities
      MotorEncoder_Update(CONTROL_DT_SEC);

      // 4. Update FSM State Machine & Cascade PID
      int16_t pwm_left = 0;
      int16_t pwm_right = 0;
      const motor_encoder_data_t *enc_data = MotorEncoder_GetData();

      RobotFSM_Update(MPU6050_GetPitch(), MPU6050_GetGyroRate(),
                      enc_data->speed_avg_mps, CONTROL_DT_SEC,
                      &pwm_left, &pwm_right);

      // 5. Output PWM to TB6612 Dual H-Bridge
      Motor_SetOutput(pwm_left, pwm_right);

      // 6. Send Telemetry if streaming is active
      if (Telemetry_IsStreaming())
      {
        Telemetry_SendData(MPU6050_GetPitch(),
                           MPU6050_GetGyroRate(),
                           (int16_t)((pwm_left + pwm_right) / 2),
                           enc_data->total_pulses_left,
                           enc_data->total_pulses_right,
                           (uint8_t)RobotFSM_GetState());
      }

      // 7. Visual LED Heartbeat
      static uint32_t s_heartbeat_cnt = 0;
      s_heartbeat_cnt++;
      if (MPU6050_IsConnected())
      {
        if (s_heartbeat_cnt % 50 == 0) // 1Hz normal heartbeat
        {
          Board_LED_Toggle();
        }
      }
      else
      {
        if (s_heartbeat_cnt % 10 == 0) // 5Hz fast error blink
        {
          Board_LED_Toggle();
        }
      }
    }
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10)
  {
    g_control_tick_10ms_flag = 1;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
