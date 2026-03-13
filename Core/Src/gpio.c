/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
/**
 * @brief 初始化I2C所需要的引脚 PB6 -> SCL, PB7 -> SDA
 * @param None
 * @return None
 */
void I2C_GPIO_Init(void)
{
  //1.使能GPIOB时钟
  *(volatile uint32_t *)(0x40023800 + 0x30) |= (1 << 1); // RCC_AHB1ENR寄存器使能GPIOB时钟

  //2.配置引脚
  //PB6，PB7的位清零，
  GPIOB->MODER &= ~( (3<<12) | (3 << 14) );
  //模式：输出模式
  GPIOB->MODER |= (1 << 12) | (1 << 14);
  //输出类型：开漏输出
  GPIOB->OTYPER |= (1 << 6) | (1 << 7);
  //输出速度
  GPIOB->OSPEEDR |= (3 << 12) | (3 << 14);
  //禁用内部上拉，因为内部的弱、慢、不符合I2C标准
  GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));
  // 设置初始电平为高（I2C 空闲状态）
  GPIOB->BSRR = (1 << 6) | (1 << 7);  
}
/* USER CODE END 2 */
