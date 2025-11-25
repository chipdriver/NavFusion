/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "a7670e.h"
#include "i2c.h"
#include "mpu9250.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /*A7076EandGNSS*/
  //AT_Getlocation_Init(); //初始化4G和GNSS模块
  /*MPU9250*/
  /* ========== 初始化 MPU9250 九轴传感器 ========== */
  int init_ret = MPU9250_9Axis_Init();
  if (init_ret == 0)
  {
      printf_uart6("MPU9250 初始化成功！\n\n");
  }
  else
  {
      printf_uart6("MPU9250 初始化失败！错误码：%d\n", init_ret);
      while(1);  // 停止运行
  }

  HAL_Delay(2000); 

  // 1. 先校准陀螺仪零偏（保持静止）
  printf_uart6("开始陀螺仪零偏校准，请保持静止 3 秒...\n");
  HAL_Delay(1000);
  MPU9250_CalibrateGyro(500, 10); // 500次采样，10ms间隔，约5秒
  printf_uart6("陀螺仪零偏校准完成！\n\n");
  
  // 2. 校准加速度计零偏（保持水平静止）
  printf_uart6("开始加速度计校准，请保持设备水平静止 2 秒...\n");
  HAL_Delay(1000);
  MPU9250_CalibrateAccel(300, 5); // 300次采样，5ms间隔，约2秒
  printf_uart6("加速度计校准完成！\n\n");
  
  // 3. 再校准磁力计（需要旋转，远离电子设备！）
  printf_uart6("开始磁力计校准，请在开阔区域旋转设备（8字形）...\n");
  HAL_Delay(1000);
  AK8963_CalibrateMag(1000, 10); // 增加到1000次采样，确保覆盖所有方向
  printf_uart6("磁力计校准完成！\n\n");
  
  // 4. 初始化 Mahony 滤波器（优化后的参数 + Anti-windup + 磁力计门控）
  MPU9250_MahonyInit(0.3f, 0.0f); // Kp=0.3, Ki=0（先关闭积分，测试纯 P 控制）
  printf_uart6("Mahony 滤波器初始化完成 (Kp=0.3, Ki=0.0)\n");
  printf_uart6("优化项：Anti-windup + 磁力计门控\n\n");
  
  float roll_deg, pitch_deg, yaw_deg;
    

  /* 数据结构体 */
  MPU9250_raw_Data mpu_raw;
  MPU9250_Physical_Data mpu_phys;
  AK8963_raw_Data ak_raw;
  AK8963_Physical_Data ak_phys;

  uint32_t mag_fail_count = 0;  // 统计磁力计失败次数
  uint32_t print_count = 0;     // 控制打印频率
  uint32_t mag_success_count = 0; // 统计磁力计成功次数
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    //AT_GNSS_GetLocation();//获取经纬度信息

    /* ========== 读取九轴数据 ========== */
    int ret = MPU9250_Read_9Axis(&mpu_raw, &mpu_phys, &ak_raw, &ak_phys);

    float dt = 0.005f; // 5ms 采样间隔
    
    if (ret == 0)
    {
        // 磁力计有效 → 9轴 Mahony
        MPU9250_MahonyUpdate(&mpu_phys, &ak_phys, dt);
        mag_fail_count = 0; // 重置失败计数
        mag_success_count++;
    }
    else
    {
        // 磁力计无效 → 用 6轴 Mahony
        MPU9250_MahonyUpdateIMU(&mpu_phys, dt);
        mag_fail_count++;
        
        // 磁力计连续失败 100 次后打印警告（0.5秒）
        if (mag_fail_count == 100) {
            printf_uart6("警告：磁力计持续失败，使用6轴融合！\n");
            mag_fail_count = 0; // 重置避免重复打印
        }
    }

    // 输出融合后的欧拉角（每 200ms 打印一次，减少串口负载）
    if (++print_count >= 40) { // 40 * 5ms = 200ms
        MPU9250_GetEulerFusedDeg(&roll_deg, &pitch_deg, &yaw_deg);
        
        // 【诊断模式】打印姿态角（静置时观察是否稳定）
        printf_uart6("姿态角：R:%.1f° P:%.1f° Y:%.1f° [Mag:%s]\n",
                     roll_deg, pitch_deg, yaw_deg,
                     (ret == 0) ? "OK" : "FAIL");
        print_count = 0;
        
        // 每 10 秒重置统计
        if (mag_success_count + mag_fail_count >= 2000) {
            mag_success_count = 0;
            mag_fail_count = 0;
        }
    }

    HAL_Delay(5);   // 约 200Hz
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
