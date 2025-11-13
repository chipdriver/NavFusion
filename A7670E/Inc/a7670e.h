#ifndef A7670E_H
#define A7670E_H

//头文件
#include "stm32f4xx_hal.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

//宏定义
#define AT_RX_BUF_SIZE 512  // 接收缓冲区大小

//外部变量
extern volatile uint32_t usart1_isr_cnt;          // USART1中断计数器

//函数声明
uint8_t AT_SendAndWait(const char *cmd, const char *expect, uint32_t timeout_ms);               // 发送AT指令并等待响应
uint8_t AT_SendFormatAndWait(const char *expect, uint32_t timeout_ms, const char *format, ...); // 发送格式化AT指令并等待响应
void AT_UART_Start_IT(void);    //使能UART接收中断
uint8_t AT_Network_Init(void);  //初始化网络
uint8_t AT_GNSS_Init(void);     //初始化GNSS模块
void AT_Getlocation_Init(void); //获取位置信息初始化
uint8_t AT_GNSS_GetLocation(void); //获取位置信息
#endif // A7670E_H