/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @note           : NavFusion 导航融合系统主程序
 *                   - MPU9250 九轴传感器姿态融合
 *                   - A7670E 4G/GNSS 定位（可选）
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

/*==============================================================================
 *                           系统头文件包含
 *============================================================================*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "a7670e.h"    // A7670E 4G/GNSS 模块
#include "i2c.h"       // I2C 软件模拟
#include "mpu9250.h"   // MPU9250 九轴传感器
#include "payload.h"    // 有关载荷控制的函数
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

/*==============================================================================
 *                           全局变量定义
 *============================================================================*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/*==============================================================================
 *                           私有函数声明
 *============================================================================*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*==============================================================================
 *                           主函数
 *============================================================================*/
/**
 * @brief  应用程序入口点
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /*============================================================================
   *                           MCU 配置
   *==========================================================================*/
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

  /*============================================================================
   *                           用户初始化代码
   *==========================================================================*/
  /* USER CODE BEGIN 2 */

  /*--- 4G+GNSS 模块初始化 ---*/
  AT_Getlocation_Init(); //初始化4G和GNSS模块

  MQTT_InitAndConnect_raw(); //初始化并连接MQTT服务器

  /*--- MPU9250 九轴传感器初始化 ---*/
  // 1. 初始化 MPU9250 九轴传感器
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

  // 2. 校准所有传感器
  MPU9250_CalibrateAll();

  // 3. 初始化 Mahony 滤波器（优化后的参数 + Anti-windup + 磁力计门控）
  MPU9250_MahonyInit(0.3f, 0.0f); // Kp=0.3, Ki=0（先关闭积分，测试纯 P 控制）
  printf_uart6("Mahony 滤波器初始化完成 (Kp=0.3, Ki=0.0)\n");
  printf_uart6("优化项：Anti-windup + 磁力计门控\n\n");

  /*--- 姿态角变量 ---*/
  float roll_deg, pitch_deg, yaw_deg;

  /*--- 数据结构体 ---*/
  MPU9250_raw_Data mpu_raw;           // MPU9250 原始数据
  MPU9250_Physical_Data mpu_phys;     // MPU9250 物理量数据
  AK8963_raw_Data ak_raw;             // AK8963 原始数据
  AK8963_Physical_Data ak_phys;       // AK8963 物理量数据

  /*--- 统计变量 ---*/
  uint32_t mag_fail_count = 0;        // 磁力计失败次数
  uint32_t print_count = 0;           // 打印频率控制
  uint32_t mag_success_count = 0;     // 磁力计成功次数

  /* USER CODE END 2 */

  /*============================================================================
   *                           主循环
   *==========================================================================*/
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /*--- GNSS 定位 ---*/
    AT_GNSS_GetLocation();//获取经纬度信息


    /*--- 读取九轴数据 ---*/
    int ret = MPU9250_Read_9Axis(&mpu_raw, &mpu_phys, &ak_raw, &ak_phys);

    float dt = 0.005f; // 5ms 采样间隔
    
    /*--- Mahony 姿态融合 ---*/
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

    /*--- 输出融合后的欧拉角（每 200ms 打印一次，减少串口负载） ---*/
    if (++print_count >= 40) 
    { // 40 * 5ms = 200ms
        MPU9250_GetEulerFusedDeg(&roll_deg, &pitch_deg, &yaw_deg);//从全局变量 g_euler_fused（弧度）转换为度数（°）
        
        // 【诊断模式】打印姿态角（静置时观察是否稳定）
        // printf_uart6("姿态角：R:%.1f° P:%.1f° Y:%.1f° [Mag:%s]\n",
        //              roll_deg, pitch_deg, yaw_deg,
        //              (ret == 0) ? "OK" : "FAIL");
        print_count = 0;
        
        // 每 10 秒重置统计
        if (mag_success_count + mag_fail_count >= 2000) 
        {
            mag_success_count = 0;
            mag_fail_count = 0;
        }
    }

    //组JSON
    char *payload = BuildPayload_WGS84_Attitude("device_id", "business_id", HAL_GetTick(), "gateway_id",
                                                  gnss_data.latitude, gnss_data.longitude, gnss_data.altitude, gnss_data.speed_knots,
                                                  roll_deg, pitch_deg, yaw_deg,
                                                  0, 0);
    if (payload != NULL)
    {
      // 先打印看看格式对不对
      printf_uart6("%s\r\n", payload);

      // ---- 下一步：MQTT 发布（你后面接 CMQTTPUB）----
      MQTT_Publish_raw("topictest", payload);

      // 用完一定 free
      free(payload);
    }

    HAL_Delay(5);   // 约 200Hz

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/*==============================================================================
 *                           系统时钟配置
 *============================================================================*/
/**
 * @brief  系统时钟配置
 * @retval None
 * @note   时钟配置：
 *         - 系统时钟源：PLL (HSI)
 *         - SYSCLK：100 MHz
 *         - HCLK：100 MHz
 *         - APB1：50 MHz
 *         - APB2：100 MHz
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
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

  /** Initializes the CPU, AHB and APB buses clocks */
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

/*==============================================================================
 *                           用户代码区
 *============================================================================*/
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*==============================================================================
 *                           错误处理函数
 *============================================================================*/
/**
 * @brief  错误处理函数
 * @retval None
 * @note   发生错误时禁用中断并进入死循环
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
 * @brief  断言失败时的报告函数
 * @param  file: 源文件名指针
 * @param  line: 断言失败的行号
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
